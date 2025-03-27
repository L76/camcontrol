#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdbool.h>
#include <time.h>
#include <GxIAPI.h>
#include <unistd.h>
#include "pixel.h"
#include "utils.h"

#include <gtk/gtk.h>
#include <gdk-pixbuf/gdk-pixbuf.h>
#include <glib/gstdio.h>

#define MEMORY_LIMIT (size_t) (4 * 1024 * 1024 * 1024)

typedef struct {
    int camId;
    GX_DEV_HANDLE device;
    PixelProcState_T *pState;
}AcqCbArg_T;

typedef struct {
    GtkWidget *recordBtn;
    GtkWidget *stopBtn;
    GtkWidget *saveBtn;
    GtkWidget *resetBtn;
    GtkWidget *expositionEntry;
    GtkWidget *gainEntry;
    GtkWidget *suffixEntry;
    GtkWidget *applyBtn;
} ButtonIface_T;


static int FramesRecorded[2] = {0, 0};
static int FramesStored[2] = {0, 0};
static struct tm CurrentDateTime = {0};
static char DirSuffix[32] = {0};

struct tm currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct = *localtime(&now);
    return tstruct;
}

GX_STATUS setTrigger(GX_DEV_HANDLE cam) {
    GX_STATUS emStatus;
    emStatus = GXSetEnum(cam, GX_ENUM_TRANSFER_CONTROL_MODE,
    GX_ENUM_TRANSFER_CONTROL_MODE_USERCONTROLED);
    //Sets the transfer operation mode to the specified transfer frame mode.
    emStatus = GXSetEnum(cam, GX_ENUM_TRANSFER_OPERATION_MODE, GX_ENUM_TRANSFER_OPERATION_MODE_MULTIBLOCK);
    //Sets the number of output frames per command.
    emStatus = GXSetInt(cam, GX_INT_TRANSFER_BLOCK_COUNT, 1);

    emStatus = GXSetEnum(cam, GX_ENUM_TRIGGER_SELECTOR, GX_ENUM_TRIGGER_SELECTOR_FRAME_START);

    emStatus = GXSetEnum(cam, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE0);

    return emStatus;
}

void rgb_bitmap_free (unsigned char *bitmap)
{
    free(bitmap);
}

GtkImage *image0, *image1;
GtkWidget *progress_bar;
GError *error = NULL;
static GQueue *frameQ[2] = {NULL};

const int resolution_w = 1920/3, resolution_h = 1200/3;

gboolean record_on = FALSE, write_on = FALSE;
gboolean WriteComplete = FALSE;
gboolean settings_changed = FALSE;

unsigned char *frame[2], *frame_shadow[2];
int frames_record_max;
double Gain = 23.0;
double Exposition = 2000;

static void button_record (GtkWidget *widget, gpointer data)
{
  g_print ("Start recording\n");
  CurrentDateTime = currentDateTime();
  ButtonIface_T *buttons = (ButtonIface_T *) data;
  gtk_widget_set_sensitive(buttons->recordBtn, false);
  record_on = TRUE;
}

static void button_stop (GtkWidget *widget, gpointer data)
{
  g_print ("Stop recording\n");
  record_on = FALSE;
  // block record button
  ButtonIface_T *buttons = (ButtonIface_T *) data;
  gtk_widget_set_sensitive(buttons->recordBtn, false);
}

static void button_save (GtkWidget *widget, gpointer data)
{
  g_print ("Saving images to disk\n");
  record_on = FALSE;
  write_on = TRUE;
  ButtonIface_T *buttons = (ButtonIface_T *) data;
  gtk_widget_set_sensitive(buttons->recordBtn, false);
}

static void button_reset (GtkWidget *widget, gpointer data)
{
    ButtonIface_T *buttons = (ButtonIface_T *) data;
    g_print ("Clearing image buffer\n");
    record_on = FALSE;
    if (!write_on) {
        g_queue_clear_full(frameQ[0], free);
        g_queue_clear_full(frameQ[1], free);
        g_print ("Frame data cleared\r\n");
        FramesRecorded[0] = 0;
        FramesRecorded[1] = 0;
        // unblock record button
        gtk_widget_set_sensitive(buttons->recordBtn, true);
    }
    //otherwise will be free in writing code
}


static void button_apply (GtkWidget *widget, gpointer data) {
    ButtonIface_T *buttons = (ButtonIface_T *) data;
    if (!record_on && !write_on) {
        const char *gainText = gtk_entry_get_text(GTK_ENTRY(buttons->gainEntry));
        const char *expositionText = gtk_entry_get_text(GTK_ENTRY(buttons->expositionEntry));

        Gain = strtod(gainText, NULL);
        Exposition = strtod(expositionText, NULL);

        const char *suffixText = gtk_entry_get_text(GTK_ENTRY(buttons->suffixEntry));
        strncpy(DirSuffix, suffixText, sizeof(DirSuffix)-1);
        settings_changed = TRUE;
    }
}

unsigned char* rgb_bitmap_allocate (int width, int height)
{
    int size = 3*width*height;
    unsigned char *buf = malloc (size);
    memset(buf, 0, size);
    return buf;
}


void update_progress_bar()
{
    const size_t smax = 128;
    char text[smax];

    //gtk_progress_bar_set_show_text (GTK_PROGRESS_BAR(progress_bar), TRUE);
    if (record_on) {
        snprintf(&text[0], smax, "Recording frames into memory: %d(%d)/%d", FramesRecorded[0], FramesRecorded[1], frames_record_max);
        gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(progress_bar), (double)FramesRecorded[0]/frames_record_max);
    } else if (write_on) {
        snprintf(&text[0], smax, "Writing images: %d(%d)/%d(%d)", FramesStored[0], FramesStored[1], FramesRecorded[0], FramesRecorded[1]);
        double fraction = (double)(FramesStored[0]+FramesStored[1])/(FramesRecorded[0]+FramesRecorded[1]);
        gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(progress_bar), fraction);
    } else
        snprintf(&text[0], smax, "Frames recorded: %d(%d)/%d", FramesRecorded[0], FramesRecorded[1], frames_record_max);

    gtk_progress_bar_set_text (GTK_PROGRESS_BAR(progress_bar), text);

}


gboolean ui_update_task (gpointer user_data)
{
    ButtonIface_T *buttons = (ButtonIface_T *) user_data;
    GdkPixbuf *pbuf;
    GBytes *raw_image;

    raw_image = g_bytes_new(frame[0], 3*resolution_w*resolution_h);
    pbuf = gdk_pixbuf_new_from_bytes (raw_image,
                                      GDK_COLORSPACE_RGB, FALSE, 8, resolution_w, resolution_h, 3*resolution_w);
    gtk_image_set_from_pixbuf (image0, pbuf);
    g_bytes_unref(raw_image);
    g_object_unref(pbuf);

    raw_image = g_bytes_new(frame[1], 3*resolution_w*resolution_h);
    pbuf = gdk_pixbuf_new_from_bytes(raw_image,
                                     GDK_COLORSPACE_RGB, FALSE, 8, resolution_w, resolution_h, 3*resolution_w);
    gtk_image_set_from_pixbuf(image1, pbuf);
    g_bytes_unref(raw_image);
    g_object_unref(pbuf);

    update_progress_bar();
    if (!gtk_widget_is_sensitive(GTK_WIDGET(buttons->recordBtn)) && WriteComplete)  {
        // return record button sensitivity
        gtk_widget_set_sensitive(GTK_WIDGET(buttons->recordBtn), true);
        WriteComplete = FALSE;
    }

    return TRUE;
}

unsigned char *getPixel(unsigned char *pixdata, int j, int i, int w, int bps) {
    return &pixdata[(j*w + i)*bps];
}

void OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM *pFrameData) {
    AcqCbArg_T *arg = pFrameData->pUserParam;

    if (!write_on) {
        //printf("Callback: cam: %u Width: %d Height: %d FrameID: %lu>\n",
        //                arg->camId, pFrameData->nWidth, pFrameData->nHeight, pFrameData->nFrameID);

        //printf("Frame status=%d\r\n", pFrameData->status);
        PixelFormatConvert(arg->pState, pFrameData);
        unsigned char *pixdata = arg->pState->RBGimageBuf;
        const int W = resolution_w * 3;
        for (int j=0; j<resolution_h; j++) {
            for (int i=0; i<resolution_w; i++)
                memcpy(getPixel(frame_shadow[arg->camId], j, i, resolution_w, 3),
                getPixel(pixdata, j*3, i*3, W, 3),
                3);
        }

        for (int j=0; j<resolution_h; j++) {
            for (int i=0; i<resolution_w; i++) {
                unsigned char *pixel = getPixel(frame_shadow[arg->camId], j, i, resolution_w, 3);
                unsigned char g = (255-pixel[0]);
                pixel[0] = g; pixel[1] = g; pixel[2] = g;
            }
        }

        //RGB24RedtoGrayscale8(frame_shadow[arg->camId], frame_shadow[arg->camId], resolution_h, resolution_w);
        SWAP(frame_shadow[arg->camId], frame[arg->camId]);
    }

    if (record_on) {
        //Grayscale version
        //RGB24toGrayscale8(arg->pState->RBGimageBuf, arg->pState->MonoImageBuf, pFrameData->nHeight, pFrameData->nWidth);
        unsigned char *frameData = malloc(pFrameData->nWidth * pFrameData->nHeight);
        if (frameData == NULL) {
            fprintf(stderr, "Memory allocation failed\r\n");
            exit(1);
        }
        fprintf(stdout, "Cam-%d frame %d\r\n", arg->camId, FramesRecorded[arg->camId]);
        RGB24RedtoGrayscale8(arg->pState->RBGimageBuf, frameData, pFrameData->nHeight, pFrameData->nWidth);
        g_queue_push_tail(frameQ[arg->camId], (gpointer) frameData);
        FramesRecorded[arg->camId] ++;
        if (FramesRecorded[arg->camId] >= frames_record_max) {
            record_on = FALSE;
        }
    }
}

void* camera_task (void * param) {

    for (int k=0; k<2; k++)
        frameQ[k] = g_queue_new();

    uint32_t nDev = 0;
    GX_STATUS status = GXUpdateDeviceList(&nDev, 1000);
    printf("Devices found:%d\r\n", nDev);

    GX_DEV_HANDLE cam[2] = {NULL};
    GX_STATUS camStatus[2] = {GX_STATUS_SUCCESS};

    camStatus[0] = GXOpenDeviceByIndex(1, &cam[0]);
    camStatus[1] = GXOpenDeviceByIndex(2, &cam[1]);
    printf("%d: %d %d\r\n", __LINE__, camStatus[0], camStatus[1]);

    size_t nSize[2] = { 0, 0 };
    camStatus[0] = GXGetStringMaxLength(cam[0], GX_STRING_DEVICE_SERIAL_NUMBER, &nSize[0]);
    camStatus[1] = GXGetStringMaxLength(cam[1], GX_STRING_DEVICE_SERIAL_NUMBER, &nSize[1]);
    printf("%d: %d %d\r\n", __LINE__, camStatus[0], camStatus[1]);

    char *idText[2];
    idText[0] = malloc(nSize[0]);
    idText[1] = malloc(nSize[1]);

    camStatus[0] = GXGetString(cam[0], GX_STRING_DEVICE_SERIAL_NUMBER, idText[0], &nSize[0]);
    camStatus[1] = GXGetString(cam[1], GX_STRING_DEVICE_SERIAL_NUMBER, idText[1], &nSize[1]);
    printf("%d: %d %d\r\n", __LINE__, camStatus[0], camStatus[1]);

    printf("CAM0-id: %s; CAM1-id: %s\r\n", idText[0], idText[1]);

    static const char cam0_id[] = "FDE24060215";
    if (strncmp(idText[0], cam0_id, MIN(sizeof(cam0_id), nSize[0]))) {
        printf("SWAPING CAMERAS!\r\n");
        SWAP(cam[0], cam[1]);
    }

    AcqCbArg_T AcqHandle[2] = {
        {.camId = 0, .device=cam[0]},
        {.camId = 1, .device=cam[1]},
    };

    int64_t colorFilter[2] = {GX_COLOR_FILTER_NONE};
    int64_t payloadSize[2]  = {0};
    for (int i=0; i<2; i++) {
        camStatus[i] = GXGetEnum(cam[i], GX_ENUM_PIXEL_COLOR_FILTER, &colorFilter[i]);
        camStatus[i] = GXGetInt(cam[i], GX_INT_PAYLOAD_SIZE, &payloadSize[i]);
        printf("CAM%d Color Filter=%ld, Payload Size=%ld\r\n", i, colorFilter[i], payloadSize[i]);
        AcqHandle[i].pState = PixelProcInit(payloadSize[i], colorFilter[i]);

        //Set exposure
        camStatus[i] = GXSetEnum(cam[i], GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
        // Use `status = GXGetFloatRange(hDevice, GX_FLOAT_EXPOSURE_TIME, &shutterRange);` to get valid range
        camStatus[i] = GXSetFloat(cam[i], GX_FLOAT_EXPOSURE_TIME, Exposition); //20ms

        //Set gain
        camStatus[i] = GXSetEnum(cam[i], GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
        camStatus[i] = GXSetFloat(cam[i], GX_FLOAT_GAIN, Gain);
    }

    if (camStatus[0] != GX_STATUS_SUCCESS ||
        camStatus[1] != GX_STATUS_SUCCESS)  {
        pthread_exit(NULL);
    }

    printf("Opened both\r\n");

    camStatus[0] = GXSetBool(cam[0], GX_BOOL_REVERSE_X, FALSE);
    camStatus[1] = GXSetBool(cam[1], GX_BOOL_REVERSE_X, TRUE);
    printf("%d REVERCE_X: %d %d\r\n", __LINE__, camStatus[0], camStatus[1]);

    camStatus[0] = GXSetBool(cam[0], GX_BOOL_REVERSE_Y, FALSE);
    camStatus[0] = GXSetBool(cam[1], GX_BOOL_REVERSE_Y, TRUE);
    printf("%d REVERSE_Y: %d %d\r\n", __LINE__, camStatus[0], camStatus[1]);

    camStatus[0] = GXSetEnum(cam[0], GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    camStatus[1] = GXSetEnum(cam[1], GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    printf("%d: Trigger Mode ON: %d %d\r\n", __LINE__, camStatus[0], camStatus[1]);

    camStatus[0] = GXSetEnum(cam[0], GX_ENUM_TRIGGER_ACTIVATION,
        GX_TRIGGER_ACTIVATION_RISINGEDGE);
    camStatus[1] = GXSetEnum(cam[1], GX_ENUM_TRIGGER_ACTIVATION,
        GX_TRIGGER_ACTIVATION_RISINGEDGE);

    camStatus[0] = GXRegisterCaptureCallback(cam[0], &AcqHandle[0], OnFrameCallbackFun);
    camStatus[1] = GXRegisterCaptureCallback(cam[1], &AcqHandle[1], OnFrameCallbackFun);

    camStatus[0] = setTrigger(cam[0]);
    camStatus[1] = setTrigger(cam[1]);

   // camStatus[0] = GXSetEnum(cam[0], GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_SINGLE_FRAME);
    //camStatus[1] = GXSetEnum(cam[1], GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_SINGLE_FRAME);

    camStatus[0] = GXSetEnum(cam[0], GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    camStatus[1] = GXSetEnum(cam[1], GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);

    camStatus[0] = GXSendCommand(cam[0], GX_COMMAND_ACQUISITION_START);
    camStatus[1] = GXSendCommand(cam[1], GX_COMMAND_ACQUISITION_START);

    while(TRUE) {
        if (write_on) {
            char dirname[64 + 1] = {0};
            strftime(dirname, 32, "%Y-%m-%d.%H:%M:%S", &CurrentDateTime);
            strncat(dirname + strlen(dirname), DirSuffix, sizeof(DirSuffix)-1);
            g_mkdir(dirname, 0777);
            FramesStored[0] = 0;
            FramesStored[1] = 0;

            // TODO: use glib functions
            for (int k=0; k<2; k++) {

                while (!g_queue_is_empty(frameQ[k])) {
                    unsigned char *frameData = g_queue_pop_head(frameQ[k]);
                    DynamicBuffer_T *io = savePngToMem(frameData, 1920, 1200);
                    char filename[64 + 1] = {0};
                    sprintf(filename, "%s/cam%d-%05d.png", dirname, k, FramesStored[k]);

                    FILE *out = fopen(filename, "wb");
                    fwrite(((DynamicBuffer_T *)io)->data, 1, ((DynamicBuffer_T *)io)->count, out);
                    fflush(out);
                    fclose(out);
                    dynamicBufferDestroy(io);
                    FramesStored[k]++;
                    free(frameData);
                }
            }
            FramesRecorded[0] = 0;
            FramesRecorded[1] = 0;
            write_on = FALSE;
            WriteComplete = TRUE;
        }
        else if (settings_changed) {
            for (int k=0; k<2; k++) {
            //Set exposure
                GXSetEnum(cam[k], GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
                // Use `status = GXGetFloatRange(hDevice, GX_FLOAT_EXPOSURE_TIME, &shutterRange);` to get valid range
                GXSetFloat(cam[k], GX_FLOAT_EXPOSURE_TIME, Exposition); //20ms
                //Set gain
                GXSetEnum(cam[k], GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
                GXSetFloat(cam[k], GX_FLOAT_GAIN, Gain);
            }
            settings_changed = FALSE;
        }
        else {
            usleep(500);
        }
    }
}


int main(int argc, char **argv) {
    frames_record_max = 3000;

    GX_STATUS status = GXInitLib();
    if (status != GX_STATUS_SUCCESS) {
        fprintf(stderr, "Could not init library:%d\r\n", status);
    }

    gtk_init (&argc, &argv);
    GtkBuilder *builder = gtk_builder_new ();

    if (gtk_builder_add_from_file (builder, "camera-control.glade", &error) == 0) {
      g_printerr ("Error loading file: %s\n", error->message);
      g_clear_error (&error);
      return 1;
    }

    ButtonIface_T buttons = {NULL};
    /* Connect signal handlers to the constructed widgets. */
    GObject *window = gtk_builder_get_object (builder, "window");
    g_signal_connect (window, "destroy", G_CALLBACK (gtk_main_quit), NULL);

    buttons.recordBtn = GTK_WIDGET(gtk_builder_get_object (builder, "button-record"));
    buttons.stopBtn = GTK_WIDGET(gtk_builder_get_object (builder, "button-stop"));
    buttons.saveBtn = GTK_WIDGET(gtk_builder_get_object (builder, "button-save"));
    buttons.resetBtn = GTK_WIDGET(gtk_builder_get_object (builder, "button-reset"));
    buttons.applyBtn = GTK_WIDGET(gtk_builder_get_object (builder, "button-apply"));

    buttons.expositionEntry = GTK_WIDGET(gtk_builder_get_object (builder, "exposition-entry"));
    buttons.gainEntry = GTK_WIDGET(gtk_builder_get_object (builder, "gain-entry"));
    buttons.suffixEntry = GTK_WIDGET(gtk_builder_get_object (builder, "suffix-entry"));

    g_signal_connect (G_OBJECT(buttons.recordBtn), "clicked", G_CALLBACK (button_record), &buttons);
    g_signal_connect (G_OBJECT(buttons.stopBtn),   "clicked", G_CALLBACK (button_stop), &buttons);
    g_signal_connect (G_OBJECT(buttons.saveBtn),   "clicked", G_CALLBACK (button_save), &buttons);
    g_signal_connect (G_OBJECT(buttons.resetBtn),  "clicked", G_CALLBACK (button_reset), &buttons);
    g_signal_connect (G_OBJECT(buttons.applyBtn),  "clicked", G_CALLBACK (button_apply), &buttons);

    progress_bar = GTK_WIDGET(gtk_builder_get_object (builder, "progress-bar"));
    gtk_progress_bar_set_show_text (GTK_PROGRESS_BAR(progress_bar), TRUE);

    image0 = GTK_IMAGE(gtk_builder_get_object (builder, "image-0"));
    image1 = GTK_IMAGE(gtk_builder_get_object (builder, "image-1"));


    frame[0] = rgb_bitmap_allocate (resolution_w, resolution_h);
    frame[1] = rgb_bitmap_allocate (resolution_w, resolution_h);
    frame_shadow[0] = rgb_bitmap_allocate (resolution_w, resolution_h);
    frame_shadow[1] = rgb_bitmap_allocate (resolution_w, resolution_h);

    gdk_threads_add_timeout(60, ui_update_task, &buttons);
    gtk_widget_show_all (GTK_WIDGET(window));

    pthread_t tid;
    pthread_attr_t attr;
    pthread_attr_init (&attr);
    pthread_create (&tid, &attr, camera_task, NULL);

    gtk_main ();

    return 0;
}
