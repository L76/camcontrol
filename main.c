#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdbool.h>
#include <time.h>
#include <GxIAPI.h>
#include <unistd.h>
#include "pixel.h"
#include "utils.h"
#include "saveimage.h"

#include <gtk/gtk.h>
#include <gdk-pixbuf/gdk-pixbuf.h>
#include <glib/gstdio.h>

#define FRAME_W 1920
#define FRAME_H 1200

typedef struct {
    int camId;
    GX_DEV_HANDLE device;
    PixelProcState_T *pState;
} AcqCbArg_T;

typedef struct {
    GtkWidget *recordBtn;
    GtkWidget *stopBtn;
    GtkWidget *resetBtn;
    GtkWidget *expositionEntry;
    GtkWidget *gainEntry;
    GtkWidget *suffixEntry;
    GtkWidget *applyBtn;
    GtkWidget* colorComboBox;
} ButtonIface_T;

typedef enum {
    MODE_GRAY, MODE_RED, MODE_GREEN, MODE_BLUE
} color_mode_t;

typedef struct {
    color_mode_t mode;
    unsigned int frames_record_max;
    double gain;
    double exposition;
    char dirname[64 + 1];
} Config_T;

static Config_T config = {
    .mode = MODE_GRAY,
    .frames_record_max = 2500,
    .gain = 23.0,
    .exposition = 2000,
    .dirname = {0}
};

typedef struct {
    uint64_t id;
    uint8_t data[FRAME_W*FRAME_H*3];
} StoredFrame_T;

typedef struct {
    uint64_t id;
    uint8_t data[FRAME_W*FRAME_H];
} StoredFrameMono_T;

typedef struct {
    uint64_t frameId;
    int camId;
} FakeFrameState_T;

static unsigned int FramesRecorded[2] = {0, 0};
static unsigned int FramesStored[2] = {0, 0};
static struct tm CurrentDateTime = {0};
static char DirSuffix[32] = {0};

static GAsyncQueue* DisplayQ[2];
static GAsyncQueue* RecordQ[2];
static GAsyncQueue* StoreQ[2];

static int CamId0 = 0;
static int CamId1 = 1;
static GX_DEV_HANDLE CamHandle[2];
static AcqCbArg_T AcqHandle[2];

struct tm currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct = *localtime(&now);
    return tstruct;
}

GtkImage *image0, *image1;
GtkWidget *progress_bar;
GError *error = NULL;

const int resolution_w = FRAME_W/3, resolution_h = FRAME_H/3;

gboolean record_on = FALSE;
gboolean settings_changed = FALSE;
gboolean record_resume = FALSE;

unsigned char *frame[2], *frame_shadow[2];

static void
button_record(GtkWidget *widget, gpointer data)
{
  g_print ("Start recording\n");
  CurrentDateTime = currentDateTime();
  ButtonIface_T *buttons = (ButtonIface_T *) data;
  gtk_widget_set_sensitive(buttons->recordBtn, FALSE);
  gtk_widget_set_sensitive(buttons->resetBtn, FALSE);
  gtk_widget_set_sensitive(buttons->applyBtn, FALSE);
  gtk_widget_set_sensitive(buttons->stopBtn, TRUE);

  if (!record_resume) {
    strftime(config.dirname, 32, "%Y-%m-%d.%H:%M:%S", &CurrentDateTime);
    strncat(config.dirname + strlen(config.dirname), DirSuffix, strnlen(DirSuffix, sizeof(DirSuffix)));
    g_mkdir(config.dirname, 0777);
    record_resume = TRUE;
  }

  record_on = TRUE;
}

static void
button_stop(GtkWidget *widget, gpointer data)
{
  g_print("Pause recording\n");
  record_on = FALSE;
  // unblock record button
  ButtonIface_T *buttons = (ButtonIface_T *) data;
  gtk_widget_set_sensitive(buttons->recordBtn, TRUE);
  gtk_widget_set_sensitive(buttons->resetBtn, TRUE);
  gtk_widget_set_sensitive(buttons->stopBtn, FALSE);
}


static void
button_reset(GtkWidget *widget, gpointer data)
{
    if (record_on)
        return;

    ButtonIface_T *buttons = (ButtonIface_T *) data;
    g_print ("Clearing image buffer\n");
    record_on = FALSE;
    record_resume = FALSE;
    if (1/*all frames written*/) {
        FramesRecorded[0] = 0;
        FramesRecorded[1] = 0;
        FramesStored[0] = 0;
        FramesStored[1] = 0;
        // unblock record button
        gtk_widget_set_sensitive(buttons->recordBtn, TRUE);
        gtk_widget_set_sensitive(buttons->applyBtn, TRUE);
        gtk_widget_set_sensitive(buttons->stopBtn, FALSE);
    }
    //otherwise will be free in writing code
}


static void
button_apply(GtkWidget *widget, gpointer data) {
    ButtonIface_T *buttons = (ButtonIface_T *) data;
    if (!record_on && !record_resume) {
        const char *gainText = gtk_entry_get_text(GTK_ENTRY(buttons->gainEntry));
        const char *expositionText = gtk_entry_get_text(GTK_ENTRY(buttons->expositionEntry));

        config.gain = strtod(gainText, NULL);
        config.exposition = strtod(expositionText, NULL);

        const char *suffixText = gtk_entry_get_text(GTK_ENTRY(buttons->suffixEntry));
        if (strlen(suffixText) > 0) {
            DirSuffix[0] = '_';
            strncpy(DirSuffix+1, suffixText, sizeof(DirSuffix)-2);
        }

        gchar *colorText = gtk_combo_box_text_get_active_text((GtkComboBoxText*)buttons->colorComboBox);
        if (!strcmp(colorText, "gray"))
            config.mode = MODE_GRAY;
        if (!strcmp(colorText, "red"))
            config.mode = MODE_RED;
        if (!strcmp(colorText, "green"))
            config.mode = MODE_GREEN;
        if (!strcmp(colorText, "blue"))
            config.mode = MODE_BLUE;
        g_print(colorText);
        g_print("\r\n");
        g_free(colorText);

        settings_changed = TRUE;
    }
}

unsigned char*
rgb_bitmap_allocate(int width, int height)
{
    int size = 3*width*height;
    unsigned char *buf = malloc (size);
    memset(buf, 0, size);
    return buf;
}


void
update_progress_bar()
{
    const size_t smax = 128;
    char text[smax];

    //gtk_progress_bar_set_show_text (GTK_PROGRESS_BAR(progress_bar), TRUE);
    if (record_on) {
        snprintf(&text[0], smax, "Recording frames: %d(%d)/%d(%d)/%d", FramesStored[0], FramesStored[1], FramesRecorded[0], FramesRecorded[1], config.frames_record_max);
        gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(progress_bar), (double)(FramesRecorded[0]+FramesRecorded[1])/(config.frames_record_max*2));
    } else {
        snprintf(&text[0], smax, "Recording frames: %d(%d)/%d(%d)/%d", FramesStored[0], FramesStored[1], FramesRecorded[0], FramesRecorded[1], config.frames_record_max);
        gtk_progress_bar_set_fraction(GTK_PROGRESS_BAR(progress_bar), (double)(FramesRecorded[0]+FramesRecorded[1])/(config.frames_record_max*2));
    }
    gtk_progress_bar_set_text (GTK_PROGRESS_BAR(progress_bar), text);
}


gboolean
ui_update_task(gpointer user_data)
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

    if (settings_changed) {
        for (int k=0; k<2; k++) {
            //Re- Set exposure
            GXSetEnum(CamHandle[k], GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
            // Use `status = GXGetFloatRange(hDevice, GX_FLOAT_EXPOSURE_TIME, &shutterRange);` to get valid range
            GXSetFloat(CamHandle[k], GX_FLOAT_EXPOSURE_TIME, config.exposition); //20ms
            //Re- Set gain
            GXSetEnum(CamHandle[k], GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
                GXSetFloat(CamHandle[k], GX_FLOAT_GAIN, config.gain);
        }
        settings_changed = FALSE;
    } else {
        if (record_on && (FramesRecorded[0] + FramesRecorded[1] >= config.frames_record_max*2))
            record_on = FALSE;
    }

    return TRUE;
}

static unsigned char*
getPixel(unsigned char *pixdata, int j, int i, int w, int bps)
{
    return &pixdata[(j*w + i)*bps];
}

void
check_cam_status_and_exit(int line, int camId, GX_STATUS camStatus)
{
    if (camStatus != GX_STATUS_SUCCESS) {
         printf("%d: camera task: CAM%d init error: %d\r\n", line, camId, camStatus);
         if (camStatus != GX_STATUS_NOT_IMPLEMENTED)
            pthread_exit(NULL);
    } else {
        printf("%d: camera task: CAM%d OK\r\n", line, camId);
    }
}

void
new_frame_push(const int camId, StoredFrame_T *new_frame)
{
    g_async_queue_push(DisplayQ[camId], g_atomic_rc_box_acquire(new_frame));

    if (record_on)
        g_async_queue_push(RecordQ[camId],  g_atomic_rc_box_acquire(new_frame));
}

void
OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM *pFrameData)
{
    AcqCbArg_T *arg = pFrameData->pUserParam;
    StoredFrame_T *new_frame = g_atomic_rc_box_new(StoredFrame_T);

    PixelFormatConvert(arg->pState, pFrameData);
    new_frame->id = pFrameData->nFrameID;
    memcpy(new_frame->data, arg->pState->RBGimageBuf, FRAME_H*FRAME_W*3);
    new_frame_push(arg->camId, new_frame);
    g_atomic_rc_box_release(new_frame);
}

void*
FakeFrameTask(void *param)
{
    FakeFrameState_T *state = param;

    const int brd = 100;
    const int sx = 200, sy = 200;
    int x = brd + rand()%(FRAME_W-2*brd-sx),
        y = brd + rand()%(FRAME_H-2*brd-sy);
    int xinc = 2, yinc = 2;


    while (1) {
        StoredFrame_T *new_frame = g_atomic_rc_box_new(StoredFrame_T);
        bzero(new_frame->data, FRAME_H*FRAME_W*3);
        new_frame->id = state->frameId;
        state->frameId += 1;

        for (int j = y; j < y + sy; j++) {
            for (int i = x; i < x + sx; i++) {
                unsigned char *pixel = getPixel(new_frame->data, j, i, FRAME_W, 3);
                pixel[0] = rand() % 256;
                pixel[1] = rand() % 256;
                pixel[2] = rand() % 256;
            }
        }

        x += xinc; if (x + sx + brd >= FRAME_W || (x < brd)) xinc = -xinc;
        y += yinc; if (y + sy + brd >= FRAME_H || (y < brd)) yinc = -yinc;

        new_frame_push(state->camId, new_frame);
        g_atomic_rc_box_release(new_frame);

        usleep(40000);
    }
}

int
init_devices()
{
    uint32_t nDev = 0;
    GX_STATUS status = GXUpdateDeviceList(&nDev, 1000);
    printf("Devices found:%d\r\n", nDev);

    if (nDev < 2) {
        return -1;
    }

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

    return 0;
}


void*
camera_start(void* param)
{
    int camId = *(int*)(param);
    AcqHandle[camId].camId = camId;
    AcqHandle[camId].device=CamHandle[camId];
    GX_STATUS  camStatus = GX_STATUS_SUCCESS;
    int64_t  colorFilter = GX_COLOR_FILTER_NONE;
    int64_t  payloadSize = 0;

    camStatus = GXGetEnum(CamHandle[camId], GX_ENUM_PIXEL_COLOR_FILTER, &colorFilter);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    camStatus = GXGetInt(CamHandle[camId], GX_INT_PAYLOAD_SIZE, &payloadSize);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    printf("CAM%d Color Filter=%ld, Payload Size=%ld\r\n", camId, colorFilter, payloadSize);

    AcqHandle[camId].pState = PixelProcInit(payloadSize, colorFilter);
    //Set exposure
    camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
    check_cam_status_and_exit(__LINE__, camId, camStatus);
    // Use `status = GXGetFloatRange(hDevice, GX_FLOAT_EXPOSURE_TIME, &shutterRange);` to get valid range
    camStatus = GXSetFloat(CamHandle[camId], GX_FLOAT_EXPOSURE_TIME, config.exposition);
    check_cam_status_and_exit(__LINE__, camId, camStatus);
    //Set gain
    camStatus =  GXSetEnum(CamHandle[camId], GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
    check_cam_status_and_exit(__LINE__, camId, camStatus);
    camStatus = GXSetFloat(CamHandle[camId], GX_FLOAT_GAIN, config.gain);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    printf("CAM %d opened\r\n", camId);

    if (camId == 0) {
        camStatus = GXSetBool(CamHandle[camId], GX_BOOL_REVERSE_X, TRUE);
        check_cam_status_and_exit(__LINE__, camId, camStatus);
        camStatus = GXSetBool(CamHandle[camId], GX_BOOL_REVERSE_Y, TRUE);
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
    camStatus = GXRegisterCaptureCallback(CamHandle[camId], &AcqHandle[camId], OnFrameCallbackFun);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    camStatus = GXSendCommand(CamHandle[camId], GX_COMMAND_ACQUISITION_START);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    return 0;
}

void*
display_q_task(void* param)
{
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
                unsigned char c;
                switch(config.mode) {
                case MODE_RED:
                    c = 255-pixel[0];
                    pixel[0] = pixel[1] = pixel[2] = c;
                    break;
                case MODE_GREEN:
                    c = 255-pixel[1];
                    pixel[0] = pixel[1] = pixel[2] = c;
                    break;
                case MODE_BLUE:
                    c = 255-pixel[2];
                    pixel[0] = pixel[1] = pixel[2] = c;
                    break;
                case MODE_GRAY:
                default:
                    unsigned int gray = pixel[0]*21268 + pixel[1]*71510 + pixel[2]*7217;
                    c = gray/100000;
                    pixel[0] = pixel[1] = pixel[2] = c;
                    break;
                }
             }
        }

        g_atomic_rc_box_release(next_frame);

        SWAP(frame_shadow[camId], frame[camId]);
    }

    return 0;
}

void*
record_q_task(void* param)
{
    int camId = *(int*)(param);
    while (1) {
        StoredFrame_T *in_frame  = g_async_queue_pop(RecordQ[camId]);
        StoredFrameMono_T *out_frame = g_atomic_rc_box_new(StoredFrameMono_T);

        out_frame->id = in_frame->id;
        fprintf(stdout, "Cam-%d frame %d actual id %ld\r\n", camId, FramesRecorded[camId], out_frame->id);

        switch(config.mode) {
        case MODE_RED:
            RGB24RedtoGrayscale8(in_frame->data, out_frame->data, FRAME_H, FRAME_W);
            break;
        case MODE_GREEN:
            RGB24GreentoGrayscale8(in_frame->data, out_frame->data, FRAME_H, FRAME_W);
            break;
        case MODE_BLUE:
            RGB24BluetoGrayscale8(in_frame->data, out_frame->data, FRAME_H, FRAME_W);
            break;
        case MODE_GRAY:
        default:
            RGB24toGrayscale8(in_frame->data, out_frame->data, FRAME_H, FRAME_W);
            break;
        }


        g_async_queue_push(StoreQ[camId], (gpointer) out_frame);
        g_atomic_rc_box_release(in_frame);
    }
}

void*
store_q_task(void* param)
{
    int camId = *(int*)(param);
    char filename[128] = {0};

    while (1) {
        StoredFrameMono_T* next_frame = g_async_queue_pop(StoreQ[camId]);
        sprintf(filename, "%s/cam%d-%05d-%08ld.png", config.dirname, camId, FramesStored[camId], next_frame->id);

        if (savePngToFile(filename, next_frame->data, FRAME_W, FRAME_H)) {
            g_printerr("unable to save png!\r\n");
        }

        FramesStored[camId]++;
        FramesRecorded[camId] = g_async_queue_length(StoreQ[camId]);
        g_atomic_rc_box_release(next_frame);
    }

    return 0;
}

typedef struct {
    pthread_t tid;
    pthread_attr_t attr;
    void* (*proc)(void*);
    void* args;
} worker_t;

worker_t worker[] = {
    { .proc = display_q_task, .args = &CamId0 },
    { .proc = display_q_task, .args = &CamId1 },
    { .proc = record_q_task,  .args = &CamId0 },
    { .proc = record_q_task,  .args = &CamId1 },
    { .proc = store_q_task,   .args = &CamId0 },
    { .proc = store_q_task,   .args = &CamId1 }
};

const unsigned int nworkers = sizeof(worker)/sizeof(worker[0]);

int
main(int argc, char **argv)
{
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
    buttons.resetBtn = GTK_WIDGET(gtk_builder_get_object (builder, "button-reset"));
    buttons.applyBtn = GTK_WIDGET(gtk_builder_get_object (builder, "button-apply"));

    buttons.expositionEntry = GTK_WIDGET(gtk_builder_get_object (builder, "exposition-entry"));
    buttons.gainEntry = GTK_WIDGET(gtk_builder_get_object (builder, "gain-entry"));
    buttons.suffixEntry = GTK_WIDGET(gtk_builder_get_object (builder, "suffix-entry"));
    buttons.colorComboBox = GTK_WIDGET(gtk_builder_get_object (builder, "combo-box-color-mode"));

    g_signal_connect (G_OBJECT(buttons.recordBtn), "clicked", G_CALLBACK (button_record), &buttons);
    g_signal_connect (G_OBJECT(buttons.stopBtn),   "clicked", G_CALLBACK (button_stop), &buttons);
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

    for (unsigned int k=0; k<2; k++) {
        StoreQ[k]   = g_async_queue_new();
        DisplayQ[k] = g_async_queue_new();
        RecordQ[k]  = g_async_queue_new();
    }

    gdk_threads_add_timeout(60, ui_update_task, &buttons);
    gtk_widget_show_all (GTK_WIDGET(window));

    // Run all worker threads
    for (unsigned int i=0; i<nworkers; i++) {
        pthread_attr_init (&worker[i].attr);
        pthread_create (&worker[i].tid, &worker[i].attr, worker[i].proc, worker[i].args);
    }

    FakeFrameState_T fakeFrameState[2];

    if (!init_devices()) {
        camera_start(&CamId0);
        camera_start(&CamId1);
    } else {
        fakeFrameState[0].camId = 0;
        fakeFrameState[0].frameId = 0;
        pthread_t tid7;
        pthread_attr_t attr7;
        pthread_attr_init (&attr7);
        pthread_create(&tid7, &attr7, FakeFrameTask, &fakeFrameState[0]);

        fakeFrameState[1].camId = 1;
        fakeFrameState[1].frameId = 0;
        pthread_t tid8;
        pthread_attr_t attr8;
        pthread_attr_init (&attr8);
        pthread_create(&tid8, &attr8, FakeFrameTask, &fakeFrameState[1]);
    }

    gtk_main();

    return 0;
}
