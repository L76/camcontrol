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
#define FRAME_W 1920
#define FRAME_H 1200

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

/*
typedef struct {
    uint8_t rgb[3];
} RGB24Pixel_T;
*/

typedef struct {
    uint64_t id;
    uint8_t data[FRAME_W*FRAME_H*3];
} StoredFrame_T;

typedef struct {
    uint64_t id;
    uint8_t data[FRAME_W*FRAME_H];
} StoredFrameMono_T;

static int FramesRecorded[2] = {0, 0};
static int FramesStored[2] = {0, 0};
static struct tm CurrentDateTime = {0};
static char DirSuffix[32] = {0};

static GAsyncQueue *DisplayQ[2];
static GAsyncQueue *RecordQ[2];
static GAsyncQueue *frameQ[2];

static const int CamId0 = 0;
static const int CamId1 = 1;
static GX_DEV_HANDLE CamHandle[2];
//static AcqCbArg_T AcqHandle[2];

struct tm currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct = *localtime(&now);
    return tstruct;
}

void rgb_bitmap_free (unsigned char *bitmap)
{
    free(bitmap);
}

GtkImage *image0, *image1;
GtkWidget *progress_bar;
GError *error = NULL;

const int resolution_w = FRAME_W/3, resolution_h = FRAME_H/3;

gboolean record_on = FALSE, write_on = FALSE;
gboolean WriteComplete = FALSE;
gboolean settings_changed = FALSE;
gboolean record_resume = FALSE;

unsigned char *frame[2], *frame_shadow[2];
int frames_record_max;
double Gain = 23.0;
double Exposition = 2000;

char dirname[64 + 1] = {0};

static void button_record (GtkWidget *widget, gpointer data)
{
  g_print ("Start recording\n");
  CurrentDateTime = currentDateTime();
  ButtonIface_T *buttons = (ButtonIface_T *) data;
  gtk_widget_set_sensitive(buttons->recordBtn, false);

  if (!record_resume) {
    strftime(dirname, 32, "%Y-%m-%d.%H:%M:%S", &CurrentDateTime);
    strncat(dirname + strlen(dirname), DirSuffix, sizeof(DirSuffix)-1);
    g_mkdir(dirname, 0777);
    record_resume = TRUE;
  }

  record_on = TRUE;
}

static void button_stop (GtkWidget *widget, gpointer data)
{
  g_print ("Pause recording\n");
  record_on = FALSE;
  // unblock record button
  ButtonIface_T *buttons = (ButtonIface_T *) data;
  gtk_widget_set_sensitive(buttons->recordBtn, true);
}

static void button_save (GtkWidget *widget, gpointer data)
{
/*
  g_print ("Saving images to disk\n");
  record_on = FALSE;
  write_on = TRUE;
  ButtonIface_T *buttons = (ButtonIface_T *) data;
  gtk_widget_set_sensitive(buttons->recordBtn, false);
*/
}

static void button_reset (GtkWidget *widget, gpointer data)
{
    ButtonIface_T *buttons = (ButtonIface_T *) data;
    g_print ("Clearing image buffer\n");
    record_on = FALSE;
    record_resume = FALSE;
    if (!write_on) {
        //g_queue_clear_full(frameQ[0], free);
        //g_queue_clear_full(frameQ[1], free);
        //g_print ("Frame data cleared\r\n");
        FramesRecorded[0] = 0;
        FramesRecorded[1] = 0;
        FramesStored[0] = 0;
        FramesStored[1] = 0;
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
        snprintf(&text[0], smax, "Recording frames: %d(%d)/%d(%d)/%d", FramesStored[0], FramesStored[1], FramesRecorded[0], FramesRecorded[1], frames_record_max);
        gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(progress_bar), (double)(FramesRecorded[0]+FramesRecorded[1])/(frames_record_max*2));
    } else {
        snprintf(&text[0], smax, "Recording frames: %d(%d)/%d(%d)/%d", FramesStored[0], FramesStored[1], FramesRecorded[0], FramesRecorded[1], frames_record_max);
        gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(progress_bar), (double)(FramesRecorded[0]+FramesRecorded[1])/(frames_record_max*2));
    }
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

    if (settings_changed) {
        for (int k=0; k<2; k++) {
            //Re- Set exposure
            GXSetEnum(CamHandle[k], GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
            // Use `status = GXGetFloatRange(hDevice, GX_FLOAT_EXPOSURE_TIME, &shutterRange);` to get valid range
            GXSetFloat(CamHandle[k], GX_FLOAT_EXPOSURE_TIME, Exposition); //20ms
            //Re- Set gain
            GXSetEnum(CamHandle[k], GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
                GXSetFloat(CamHandle[k], GX_FLOAT_GAIN, Gain);
        }
        settings_changed = FALSE;
    } else {
        if (record_on && (FramesRecorded[0] + FramesRecorded[1] >= frames_record_max*2))
            record_on = FALSE;
    }

    return TRUE;
}

unsigned char *getPixel(unsigned char *pixdata, int j, int i, int w, int bps) {
    return &pixdata[(j*w + i)*bps];
}

void check_cam_status_and_exit(int line, int camId, GX_STATUS  camStatus) {
    if (camStatus != GX_STATUS_SUCCESS) {
         printf("%d: camera task: CAM%d init error: %d\r\n", line, camId, camStatus);
         if (camStatus != GX_STATUS_NOT_IMPLEMENTED)
            pthread_exit(NULL);
    } else {
        printf("%d: camera task: CAM%d OK\r\n", line, camId);
    }
}

void OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM *pFrameData) {
    AcqCbArg_T *arg = pFrameData->pUserParam;
    StoredFrame_T *new_frame = g_atomic_rc_box_new(StoredFrame_T);

    PixelFormatConvert(arg->pState, pFrameData);
    new_frame->id = pFrameData->nFrameID;
    memcpy(new_frame->data, arg->pState->RBGimageBuf, FRAME_H*FRAME_W*3);
    if (!write_on)
        g_async_queue_push(DisplayQ[arg->camId], g_atomic_rc_box_acquire(new_frame));

    if (record_on)
        g_async_queue_push(RecordQ[arg->camId],   g_atomic_rc_box_acquire(new_frame));

    g_atomic_rc_box_release(new_frame);

    //For single frame asq. test
    //GX_STATUS camStatus = GXSendCommand(CamHandle[arg->camId], GX_COMMAND_ACQUISITION_START);
    //check_cam_status_and_exit(__LINE__, arg->camId, camStatus);
}

void init_devices() {
    uint32_t nDev = 0;
    GX_STATUS status = GXUpdateDeviceList(&nDev, 1000);
    printf("Devices found:%d\r\n", nDev);

    GX_STATUS camStatus[2] = {GX_STATUS_SUCCESS};

    for (int i = 0; i < 2; i++)
        camStatus[i] = GXOpenDeviceByIndex(i+1, &CamHandle[i]);

    printf("%d: %d %d\r\n", __LINE__, camStatus[0], camStatus[1]);

    size_t nSize[2] = { 0, 0 };
    for (int i = 0; i < 2; i++)
        camStatus[i] = GXGetStringMaxLength(CamHandle[i], GX_STRING_DEVICE_SERIAL_NUMBER, &nSize[i]);

    printf("%d: %d %d\r\n", __LINE__, camStatus[0], camStatus[1]);

    char *idText[2];
    for (int i = 0; i < 2; i++)
        idText[i] = malloc(nSize[i]);

    for (int i = 0; i < 2; i++)
        camStatus[i] = GXGetString(CamHandle[i], GX_STRING_DEVICE_SERIAL_NUMBER, idText[i], &nSize[i]);

    printf("%d: %d %d\r\n", __LINE__, camStatus[0], camStatus[1]);
    printf("CAM0-id: %s; CAM1-id: %s\r\n", idText[0], idText[1]);

    static const char cam0_id[] = "FDE24060215";
    if (strncmp(idText[0], cam0_id, MIN(sizeof(cam0_id), nSize[0]))) {
        printf("SWAPING CAMERAS!\r\n");
        SWAP(CamHandle[0], CamHandle[1]);
    }

    for (int i = 0; i < 2; i++)
        free(idText[i]);
}



void* camera_task (void *param) {
    int camId = *(int*)(param);
    AcqCbArg_T acqHandle = {.camId = camId, .device=CamHandle[camId]};
    GX_STATUS  camStatus = GX_STATUS_SUCCESS;
    int64_t  colorFilter = GX_COLOR_FILTER_NONE;
    int64_t  payloadSize = 0;

    camStatus = GXGetEnum(CamHandle[camId], GX_ENUM_PIXEL_COLOR_FILTER, &colorFilter);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    camStatus = GXGetInt(CamHandle[camId], GX_INT_PAYLOAD_SIZE, &payloadSize);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    printf("CAM%d Color Filter=%ld, Payload Size=%ld\r\n", camId, colorFilter, payloadSize);

    acqHandle.pState = PixelProcInit(payloadSize, colorFilter);
    //Set exposure
    camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
    check_cam_status_and_exit(__LINE__, camId, camStatus);
    // Use `status = GXGetFloatRange(hDevice, GX_FLOAT_EXPOSURE_TIME, &shutterRange);` to get valid range
    camStatus = GXSetFloat(CamHandle[camId], GX_FLOAT_EXPOSURE_TIME, Exposition);
    check_cam_status_and_exit(__LINE__, camId, camStatus);
    //Set gain
    camStatus =  GXSetEnum(CamHandle[camId], GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
    check_cam_status_and_exit(__LINE__, camId, camStatus);
    camStatus = GXSetFloat(CamHandle[camId], GX_FLOAT_GAIN, Gain);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    printf("CAM %d opened\r\n", camId);

    if (camId == 0) {
        camStatus = GXSetBool(CamHandle[camId], GX_BOOL_REVERSE_X, FALSE);
        check_cam_status_and_exit(__LINE__, camId, camStatus);
        camStatus = GXSetBool(CamHandle[camId], GX_BOOL_REVERSE_Y, FALSE);
        check_cam_status_and_exit(__LINE__, camId, camStatus);
    }

    if (camId == 1) {
        camStatus = GXSetBool(CamHandle[camId], GX_BOOL_REVERSE_X, TRUE);
        check_cam_status_and_exit(__LINE__, camId, camStatus);
        camStatus = GXSetBool(CamHandle[camId], GX_BOOL_REVERSE_Y, TRUE);
        check_cam_status_and_exit(__LINE__, camId, camStatus);
    }

    camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_TRIGGER_ACTIVATION, GX_TRIGGER_ACTIVATION_RISINGEDGE);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    // Trigger configuration start
    camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_TRANSFER_CONTROL_MODE, GX_ENUM_TRANSFER_CONTROL_MODE_USERCONTROLED);
    // or GX_ENUM_TRANSFER_CONTROL_MODE_BASIC
    //camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_TRANSFER_CONTROL_MODE, GX_ENUM_TRANSFER_CONTROL_MODE_BASIC);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    //Sets the transfer operation mode to the specified transfer frame mode.
    camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_TRANSFER_OPERATION_MODE, GX_ENUM_TRANSFER_OPERATION_MODE_MULTIBLOCK);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    //Sets the number of output frames per command.
    camStatus = GXSetInt(CamHandle[camId], GX_INT_TRANSFER_BLOCK_COUNT, 1);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_TRIGGER_SELECTOR, GX_ENUM_TRIGGER_SELECTOR_FRAME_START);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE0);
    check_cam_status_and_exit(__LINE__, camId, camStatus);
    // End of trigger configuration

    //Register Callback
    camStatus = GXRegisterCaptureCallback(CamHandle[camId], &acqHandle, OnFrameCallbackFun);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    //camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_SINGLE_FRAME);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    camStatus = GXSendCommand(CamHandle[camId], GX_COMMAND_ACQUISITION_START);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    // The thread must not die!
    while(1) { usleep(1); }
}

void* display_q_task (void *param) {
    int camId = *(int*)(param);
    while (1) {
        StoredFrame_T *next_frame = g_async_queue_pop(DisplayQ[camId]);

        const int W = resolution_w * 3;
        for (int j=0; j<resolution_h; j++) {
            for (int i=0; i<resolution_w; i++)
                memcpy(getPixel(frame_shadow[camId], j, i, resolution_w, 3),
                                      getPixel(next_frame->data, j*3, i*3, W, 3),
                                                                          3);
        }

        for (int j=0; j<resolution_h; j++) {
            for (int i=0; i<resolution_w; i++) {
                unsigned char *pixel = getPixel(frame_shadow[camId], j, i, resolution_w, 3);
                unsigned char g = 255-pixel[1];
                pixel[0] = g; pixel[1] = g; pixel[2] = g;
             }
        }

        //printf("display_Q %d: release %d\n", camId, next_frame->id);
        g_atomic_rc_box_release(next_frame);

        SWAP(frame_shadow[camId], frame[camId]);
    }
}

void* record_q_task (void *param) {
    int camId = *(int*)(param);
    while (1) {
        StoredFrame_T *in_frame  = g_async_queue_pop(RecordQ[camId]);
        StoredFrameMono_T *out_frame = g_atomic_rc_box_new(StoredFrameMono_T);

        out_frame->id = in_frame->id;
        fprintf(stdout, "Cam-%d frame %d actual id %lld\r\n", camId, FramesRecorded[camId], out_frame->id);

        RGB24GreentoGrayscale8(in_frame->data, out_frame->data, FRAME_H, FRAME_W);
        g_async_queue_push(frameQ[camId], (gpointer) out_frame);
        g_atomic_rc_box_release(in_frame);
    }
}

void* store_q_task (void *param) {
    int camId = *(int*)(param);
    while (1) {
        StoredFrameMono_T *next_frame = g_async_queue_pop(frameQ[camId]);
        uint8_t *frameData = next_frame->data;
        uint64_t frame_id = next_frame->id;

        DynamicBuffer_T *io = savePngToMem(frameData, FRAME_W, FRAME_H);
        char filename[64 + 9 + 1] = {0};
        sprintf(filename, "%s/cam%d-%05d-%08lld.png", dirname, camId, FramesStored[camId], frame_id);

        FILE *out = fopen(filename, "wb");
        fwrite(((DynamicBuffer_T *)io)->data, 1, ((DynamicBuffer_T *)io)->count, out);
        fflush(out);
        fclose(out);
        dynamicBufferDestroy(io);

        FramesStored[camId]++;
        FramesRecorded[camId] = g_async_queue_length(frameQ[camId]);
        g_atomic_rc_box_release(next_frame);
    }
}

int main(int argc, char **argv) {
    frames_record_max = 2500;

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

    for (int k=0; k<2; k++) {
        frameQ[k]   = g_async_queue_new();
        DisplayQ[k] = g_async_queue_new();
        RecordQ[k]  = g_async_queue_new();
    }

    gdk_threads_add_timeout(60, ui_update_task, &buttons);
    gtk_widget_show_all (GTK_WIDGET(window));

    pthread_t tid1;
    pthread_attr_t attr1;
    pthread_attr_init (&attr1);
    pthread_create (&tid1, &attr1, display_q_task, &CamId0);

    pthread_t tid2;
    pthread_attr_t attr2;
    pthread_attr_init (&attr2);
    pthread_create (&tid2, &attr2, display_q_task, &CamId1);

    pthread_t tid3;
    pthread_attr_t attr3;
    pthread_attr_init (&attr3);
    pthread_create (&tid3, &attr3, record_q_task, &CamId0);

    pthread_t tid4;
    pthread_attr_t attr4;
    pthread_attr_init (&attr4);
    pthread_create (&tid4, &attr4, record_q_task, &CamId1);

    pthread_t tid5;
    pthread_attr_t attr5;
    pthread_attr_init (&attr5);
    pthread_create (&tid5, &attr5, store_q_task, &CamId0);

    pthread_t tid6;
    pthread_attr_t attr6;
    pthread_attr_init (&attr6);
    pthread_create (&tid6, &attr6, store_q_task, &CamId1);

    //Pre-init cameras
    init_devices();

    // Run acquisition threads
    pthread_t tid00;
    pthread_attr_t attr00;
    pthread_attr_init (&attr00);
    pthread_create (&tid00, &attr00, camera_task, &CamId0);

    pthread_t tid01;
    pthread_attr_t attr01;
    pthread_attr_init (&attr01);
    pthread_create (&tid01, &attr01, camera_task, &CamId1);
    gtk_main();

    return 0;
}
