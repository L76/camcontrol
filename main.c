#include "std.h"
#include "galaxy.h"
#include "pixel.h"
#include "utils.h"
#include "saveimage.h"

#include <gtk/gtk.h>
#include <glib/gstdio.h>
#include <glib.h>

//#define FRAME_W 1920
//#define FRAME_H 1200

typedef struct {
    int camId;
    GX_DEV_HANDLE device;
    PixelProcState_T *pState;
} AcqCbArg_T;

typedef struct {
    GtkWidget* recordBtn;
    GtkWidget* stopBtn;
    GtkWidget* resetBtn;
    GtkWidget* expositionEntry[2];
    GtkWidget* gainEntry[2];
    GtkWidget* framewEntry[2];
    GtkWidget* framehEntry[2];
    GtkWidget* offsetxEntry[2];
    GtkWidget* offsetyEntry[2];
    GtkWidget* suffixEntry;
    GtkWidget* applyBtn;
    GtkWidget* colorComboBox[2];
    GtkWidget* acqStartBtn;
    GtkWidget* acqStopBtn;
    GtkWidget* swapDevBtn;
    GtkWidget* hMirror[2];
    GtkWidget* wMirror[2];
} ButtonIface_T;

typedef enum {
    MODE_GRAY, MODE_RED, MODE_GREEN, MODE_BLUE
} color_mode_t;

typedef struct {
    uint64_t frames_record_max;
    uint64_t camwmax[2], camhmax[2];
    color_mode_t mode[2];
    double gain[2];
    double exposition[2];
    unsigned int framew[2], frameh[2];
    unsigned int offsetx[2], offsety[2];
    char dirname[64 + 1];
} Config_T;

static Config_T config = {
    .mode = {MODE_GRAY, MODE_GRAY},
    .frames_record_max = 2500,
    .gain = {23.0, 23.0},
    .exposition = {2000, 2000},
    .framew = {1920, 1920},
    .frameh = {1200, 1200},
    .offsetx = {0, 0},
    .offsety = {0, 0},
    .dirname = {0}
};

typedef struct {
    uint64_t id;
    uint32_t h, w;
    uint32_t bpp;
} StoredFrameD_T;

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
static FakeFrameState_T fakeFrameState[2];

struct tm currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct = *localtime(&now);
    return tstruct;
}

GtkImage* image[2];
GtkWidget *progress_bar;
GError *error = NULL;

gboolean acq_enable = FALSE;
gboolean cams_swapped = FALSE;
gboolean fake_cameras = FALSE;
gboolean record_on = FALSE;
gboolean record_resume = FALSE;

StoredFrameD_T *frame[2], *frame_shadow[2];


uint8_t*
sframedata(StoredFrameD_T* sf) {
    return (uint8_t*)sf + sizeof(StoredFrameD_T);
}


uint32_t
sframedatasize(StoredFrameD_T* sf) {
    return sf->w * sf->h * sf->bpp;
}


static StoredFrameD_T*
sframealloc(int32_t h, int32_t w, int32_t bpp) {
    StoredFrameD_T* sf = g_atomic_rc_box_alloc(sizeof(StoredFrameD_T) + h*w*bpp);
    sf->h = h;
    sf->w = w;
    sf->bpp = bpp;
    memset(sframedata(sf), 0, sframedatasize(sf));
    return sf;
}


static void
sframefree(StoredFrameD_T* sf) {
    g_atomic_rc_box_release(sf);
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
new_frame_push(const int camId, StoredFrameD_T* new_frame)
{
    g_async_queue_push(DisplayQ[camId], g_atomic_rc_box_acquire(new_frame));

    if (record_on)
        g_async_queue_push(RecordQ[camId],  g_atomic_rc_box_acquire(new_frame));
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

    for (int i = 0; i < 2; i++)
        free(idText[i]);

    return 0;
}


void
OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM *pFrameData)
{
    AcqCbArg_T* arg = pFrameData->pUserParam;
    StoredFrameD_T* new_frame = sframealloc(config.frameh[0], config.framew[0], 3);

    PixelFormatConvert(arg->pState, pFrameData);
    new_frame->id = pFrameData->nFrameID;
    memcpy(sframedata(new_frame), arg->pState->RBGimageBuf, sframedatasize(new_frame));
    new_frame_push(arg->camId, new_frame);
    g_atomic_rc_box_release(new_frame);
}


void*
camera_configure(void* param)
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
    camStatus = GXSetFloat(CamHandle[camId], GX_FLOAT_EXPOSURE_TIME, config.exposition[camId]);
    check_cam_status_and_exit(__LINE__, camId, camStatus);
    //Set gain
    camStatus =  GXSetEnum(CamHandle[camId], GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
    check_cam_status_and_exit(__LINE__, camId, camStatus);
    camStatus = GXSetFloat(CamHandle[camId], GX_FLOAT_GAIN, config.gain[camId]);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    camStatus = GXGetInt(CamHandle[camId], GX_INT_SENSOR_WIDTH, &config.camwmax[camId]);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    camStatus = GXGetInt(CamHandle[camId], GX_INT_SENSOR_HEIGHT, &config.camhmax[camId]);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    printf("CAM %d opened\r\n", camId);

    camStatus = GXSetBool(CamHandle[camId], GX_BOOL_REVERSE_X, FALSE);
    check_cam_status_and_exit(__LINE__, camId, camStatus);
    camStatus = GXSetBool(CamHandle[camId], GX_BOOL_REVERSE_Y, FALSE);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_TRIGGER_ACTIVATION, GX_TRIGGER_ACTIVATION_RISINGEDGE);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

/*
    // PROBABLY NOT IMPLEMENTED:
    // Trigger configuration start
    camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_TRANSFER_CONTROL_MODE, GX_ENUM_TRANSFER_CONTROL_MODE_USERCONTROLED);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    //Sets the transfer operation mode to the specified transfer frame mode.
    camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_TRANSFER_OPERATION_MODE, GX_ENUM_TRANSFER_OPERATION_MODE_MULTIBLOCK);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    //Sets the number of output frames per command.
    camStatus = GXSetInt(CamHandle[camId], GX_INT_TRANSFER_BLOCK_COUNT, 1);
    check_cam_status_and_exit(__LINE__, camId, camStatus);
*/

    camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_TRIGGER_SELECTOR, GX_ENUM_TRIGGER_SELECTOR_FRAME_START);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE0);
    check_cam_status_and_exit(__LINE__, camId, camStatus);
    // End of trigger configuration

    return 0;
}


static void
camera_change_settings()
{
    GX_STATUS  camStatus = GX_STATUS_SUCCESS;

    for (int k=0; k<2; k++) {
        //Re- Set exposure
        GXSetEnum(CamHandle[k], GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED);
        // Use `status = GXGetFloatRange(hDevice, GX_FLOAT_EXPOSURE_TIME, &shutterRange);` to get valid range
        GXSetFloat(CamHandle[k], GX_FLOAT_EXPOSURE_TIME, config.exposition[k]); //20ms
        //Re- Set gain
        GXSetEnum(CamHandle[k], GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
        GXSetFloat(CamHandle[k], GX_FLOAT_GAIN, config.gain[k]);

        GXSetInt(CamHandle[k], GX_INT_WIDTH,    config.framew[k]);
        GXSetInt(CamHandle[k], GX_INT_HEIGHT,   config.frameh[k]);
        GXSetInt(CamHandle[k], GX_INT_OFFSET_X, config.offsetx[k]);
        GXSetInt(CamHandle[k], GX_INT_OFFSET_Y, config.offsety[k]);

        g_print("reset ROI\n");
        camStatus = GXGetInt(CamHandle[k], GX_INT_WIDTH_MAX, &config.camwmax[k]);
        check_cam_status_and_exit(__LINE__, k, camStatus);

        camStatus = GXGetInt(CamHandle[k], GX_INT_HEIGHT_MAX, &config.camhmax[k]);
        check_cam_status_and_exit(__LINE__, k, camStatus);

        printf("NEW WH: %d %d\n",config.camwmax[k], config.camhmax[k] );
        //!!!!
        sframefree(frame[k]);
        frame[k] = sframealloc(config.camhmax[k]/3, config.camwmax[k]/3, 3);

        sframefree(frame_shadow[k]);
        frame_shadow[k] = sframealloc(config.camhmax[k]/3, config.camwmax[k]/3, 3);
    }
}

void*
camera_acq_start(void* param)
{
    int camId = *(int*)(param);
    AcqHandle[camId].camId = camId;
    AcqHandle[camId].device = CamHandle[camId];
    GX_STATUS  camStatus = GX_STATUS_SUCCESS;

    camStatus = GXRegisterCaptureCallback(CamHandle[camId], &AcqHandle[camId], OnFrameCallbackFun);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    camStatus = GXSetEnum(CamHandle[camId], GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    camStatus = GXSendCommand(CamHandle[camId], GX_COMMAND_ACQUISITION_START);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    if (camStatus != GX_STATUS_SUCCESS)
        return (void*)1;

    return 0;
}


void*
camera_acq_stop(void* param)
{
    int camId = *(int*)(param);
    GX_STATUS  camStatus = GX_STATUS_SUCCESS;

    camStatus = GXSendCommand(CamHandle[camId], GX_COMMAND_ACQUISITION_STOP);
    check_cam_status_and_exit(__LINE__, camId, camStatus);

    if (camStatus != GX_STATUS_SUCCESS)
        return (void*)1;

    return 0;
}


static void
acq_full_enable()
{
    acq_enable = TRUE;
    if (!fake_cameras && (camera_acq_start(&CamId0) || camera_acq_start(&CamId1)))
        acq_enable = FALSE;
}

static void
acq_full_stop()
{
    acq_enable = FALSE;
    if (!fake_cameras && (camera_acq_stop(&CamId0) || camera_acq_stop(&CamId1)))
        acq_enable = TRUE;
}


void*
FakeFrameTask(void *param)
{
    FakeFrameState_T *state = param;

    const int brd = 100;
    const int sx = 200, sy = 200;
    int x = brd + rand()%(config.framew[0]-2*brd-sx),
        y = brd + rand()%(config.frameh[0]-2*brd-sy);
    int xinc = 2, yinc = 2;


    while (1) {
        if (!acq_enable) {
            usleep(100000);
            continue;
        }

        StoredFrameD_T* new_frame = sframealloc(config.frameh[0], config.framew[0], 3);
        bzero(sframedata(new_frame), sframedatasize(new_frame));
        new_frame->id = state->frameId;
        state->frameId += 1;

        for (int j = y; j < y + sy; j++) {
            for (int i = x; i < x + sx; i++) {
                unsigned char *pixel = getPixel(sframedata(new_frame), j, i, new_frame->w, new_frame->bpp);
                pixel[0] = rand() % 256;
                pixel[1] = rand() % 256;
                pixel[2] = rand() % 256;
            }
        }

        x += xinc; if (x + sx + brd >= new_frame->w || (x < brd)) xinc = -xinc;
        y += yinc; if (y + sy + brd >= new_frame->h || (y < brd)) yinc = -yinc;

        new_frame_push(state->camId, new_frame);
        g_atomic_rc_box_release(new_frame);

        usleep(40000);
    }
}


static void
button_acq_start(GtkWidget *widget, gpointer data)
{
    ButtonIface_T *buttons = (ButtonIface_T *) data;

    if (!acq_enable) {
        g_print("Starting acquisition \n");
        camera_change_settings();
        acq_full_enable();
    } else {
        g_print("Acquisition already running \n");
    }
}


static void
button_acq_stop(GtkWidget *widget, gpointer data)
{
    if (acq_enable) {
        g_print("Stopping acquisition \n");
        acq_full_stop();
    } else {
        g_print("Acquisition not running \n");
    }
}

static void
button_mirror_state(GtkWidget *widget, gpointer data)
{
    ButtonIface_T *buttons = (ButtonIface_T *) data;
    GX_STATUS  camStatus = GX_STATUS_SUCCESS;

    if (fake_cameras)
        return;

    for (int camId = 0; camId < 2; camId++) {
        if (gtk_toggle_button_get_active((GtkToggleButton*)buttons->hMirror[camId])) {
            camStatus = GXSetBool(CamHandle[camId], GX_BOOL_REVERSE_Y, TRUE);
            check_cam_status_and_exit(__LINE__, camId, camStatus);
        } else {
            camStatus = GXSetBool(CamHandle[camId], GX_BOOL_REVERSE_Y, FALSE);
            check_cam_status_and_exit(__LINE__, camId, camStatus);
        }

        if (gtk_toggle_button_get_active((GtkToggleButton*)buttons->wMirror[camId])) {
            camStatus = GXSetBool(CamHandle[camId], GX_BOOL_REVERSE_X, TRUE);
            check_cam_status_and_exit(__LINE__, camId, camStatus);
        } else {
            camStatus = GXSetBool(CamHandle[camId], GX_BOOL_REVERSE_X, FALSE);
            check_cam_status_and_exit(__LINE__, camId, camStatus);
        }
    }
}


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
    strftime(config.dirname, 32, "%Y-%m-%d_%H%M%S", &CurrentDateTime);
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
        acq_full_stop();
        for (unsigned int k = 0; k < 2; k++) {
            const char* gainText       = gtk_entry_get_text(GTK_ENTRY(buttons->gainEntry[k]));
            const char* expositionText = gtk_entry_get_text(GTK_ENTRY(buttons->expositionEntry[k]));
            const char* framewText     = gtk_entry_get_text(GTK_ENTRY(buttons->framewEntry[k]));
            const char* framehText     = gtk_entry_get_text(GTK_ENTRY(buttons->framehEntry[k]));
            const char* offsetxText    = gtk_entry_get_text(GTK_ENTRY(buttons->offsetxEntry[k]));
            const char* offsetyText    = gtk_entry_get_text(GTK_ENTRY(buttons->offsetxEntry[k]));

            config.gain[k]       = strtod(gainText, NULL);
            config.exposition[k] = strtod(expositionText, NULL);
            config.framew[k]     = strtod(framewText, NULL);
            config.frameh[k]     = strtod(framehText, NULL);
            config.offsetx[k]    = strtod(offsetxText, NULL);
            config.offsety[k]    = strtod(offsetyText, NULL);

            gchar *colorText = gtk_combo_box_text_get_active_text((GtkComboBoxText*)buttons->colorComboBox[k]);
            if (!strcmp(colorText, "gray"))
                config.mode[k] = MODE_GRAY;
            if (!strcmp(colorText, "red"))
                config.mode[k] = MODE_RED;
            if (!strcmp(colorText, "green"))
                config.mode[k] = MODE_GREEN;
            if (!strcmp(colorText, "blue"))
                config.mode[k] = MODE_BLUE;
            g_free(colorText);

            if (gtk_toggle_button_get_active((GtkToggleButton*)buttons->swapDevBtn) && !cams_swapped) {
                g_print("Swapping cameras\n");
                SWAP(CamHandle[0], CamHandle[1]);
                cams_swapped = TRUE;
             }

            if (!gtk_toggle_button_get_active((GtkToggleButton*)buttons->swapDevBtn) && cams_swapped) {
                g_print("Swapping cameras\n");
                SWAP(CamHandle[0], CamHandle[1]);
                cams_swapped = FALSE;
            }
        }

        const char *suffixText = gtk_entry_get_text(GTK_ENTRY(buttons->suffixEntry));
        if (strlen(suffixText) > 0) {
            DirSuffix[0] = '_';
            strncpy(DirSuffix+1, suffixText, sizeof(DirSuffix)-2);
        }

        camera_change_settings();
        acq_full_enable();
    }
}

/*
unsigned char*
rgb_bitmap_allocate(int width, int height)
{
    int size = 3*width*height;
    unsigned char *buf = malloc (size);
    memset(buf, 0, size);
    return buf;
}
*/

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

    for (unsigned int k=0; k<2; k++) {
        raw_image = g_bytes_new(sframedata(frame[k]), sframedatasize(frame[k]));
        pbuf = gdk_pixbuf_new_from_bytes(raw_image, GDK_COLORSPACE_RGB, FALSE, 8, frame[k]->w, frame[k]->h, frame[k]->bpp*frame[k]->w);
        gtk_image_set_from_pixbuf(image[k], pbuf);
        g_bytes_unref(raw_image);
        g_object_unref(pbuf);
    }

/*
    raw_image = g_bytes_new(frame[1], 3*resolution_w*resolution_h);
    pbuf = gdk_pixbuf_new_from_bytes(raw_image, GDK_COLORSPACE_RGB, FALSE, 8, resolution_w, resolution_h, 3*resolution_w);
    gtk_image_set_from_pixbuf(image[1], pbuf);
    g_bytes_unref(raw_image);
    g_object_unref(pbuf);
*/

    update_progress_bar();

    if (record_on && (FramesRecorded[0] + FramesRecorded[1] >= config.frames_record_max*2))
        record_on = FALSE;

    return TRUE;
}


void*
display_q_task(void* param)
{
    int camId = *(int*)(param);
    while (1) {
        StoredFrameD_T* next_frame = g_async_queue_pop(DisplayQ[camId]);

        for (int j=0; j<frame_shadow[camId]->h; j++) {
            for (int i=0; i<frame_shadow[camId]->w; i++)
                memcpy(getPixel(sframedata(frame_shadow[camId]), j, i, frame_shadow[camId]->w, frame_shadow[camId]->bpp),
                                      getPixel(sframedata(next_frame), j*3, i*3, next_frame->w, next_frame->bpp),
                                                                          frame_shadow[camId]->bpp);
        }

        for (int j=0; j<frame_shadow[camId]->h; j++) {
            for (int i=0; i<frame_shadow[camId]->w; i++) {
                unsigned char *pixel = getPixel(sframedata(frame_shadow[camId]), j, i, frame_shadow[camId]->w, frame_shadow[camId]->bpp);
                unsigned char c;
                switch(config.mode[camId]) {
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
        StoredFrameD_T* in_frame  = g_async_queue_pop(RecordQ[camId]);
        StoredFrameD_T* out_frame = sframealloc(in_frame->h, in_frame->w, 1);

        out_frame->id = in_frame->id;
        //fprintf(stdout, "Cam-%d frame %d actual id %ld\r\n", camId, FramesRecorded[camId], out_frame->id);

        switch(config.mode[camId]) {
        case MODE_RED:
            RGB24RedtoGrayscale8(sframedata(in_frame), sframedata(out_frame), in_frame->h, in_frame->w);
            break;
        case MODE_GREEN:
            RGB24GreentoGrayscale8(sframedata(in_frame), sframedata(out_frame), in_frame->h, in_frame->w);
            break;
        case MODE_BLUE:
            RGB24BluetoGrayscale8(sframedata(in_frame), sframedata(out_frame), in_frame->h, in_frame->w);
            break;
        case MODE_GRAY:
        default:
            RGB24toGrayscale8(sframedata(in_frame), sframedata(out_frame), in_frame->h, in_frame->w);
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
        //StoredFrameMono_T* next_frame = g_async_queue_pop(StoreQ[camId]);
        StoredFrameD_T* next_frame = g_async_queue_pop(StoreQ[camId]);
        sprintf(filename, "%s/cam%d-%05d-%08ld.png", config.dirname, camId, FramesStored[camId], next_frame->id);

        if (savePngToFile(filename, sframedata(next_frame), next_frame->w, next_frame->h)) {
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
    GtkBuilder *builder = gtk_builder_new();

    if (gtk_builder_add_from_file(builder, "camera-control.glade", &error) == 0) {
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
    buttons.acqStartBtn = GTK_WIDGET(gtk_builder_get_object (builder, "button-acq-start"));
    buttons.acqStopBtn = GTK_WIDGET(gtk_builder_get_object (builder, "button-acq-stop"));
    buttons.swapDevBtn = GTK_WIDGET(gtk_builder_get_object (builder, "button-swap-devices"));
    buttons.hMirror[0] = GTK_WIDGET(gtk_builder_get_object (builder, "button-h-mirror-0"));
    buttons.wMirror[0] = GTK_WIDGET(gtk_builder_get_object (builder, "button-w-mirror-0"));
    buttons.hMirror[1] = GTK_WIDGET(gtk_builder_get_object (builder, "button-h-mirror-1"));
    buttons.wMirror[1] = GTK_WIDGET(gtk_builder_get_object (builder, "button-w-mirror-1"));

    buttons.expositionEntry[0] = GTK_WIDGET(gtk_builder_get_object (builder, "exposition-entry-0"));
    buttons.expositionEntry[1] = GTK_WIDGET(gtk_builder_get_object (builder, "exposition-entry-1"));
    buttons.gainEntry[0] = GTK_WIDGET(gtk_builder_get_object (builder, "gain-entry-0"));
    buttons.gainEntry[1] = GTK_WIDGET(gtk_builder_get_object (builder, "gain-entry-1"));
    buttons.colorComboBox[0] = GTK_WIDGET(gtk_builder_get_object (builder, "combo-box-color-mode-0"));
    buttons.colorComboBox[1] = GTK_WIDGET(gtk_builder_get_object (builder, "combo-box-color-mode-1"));
    buttons.framewEntry[0] = GTK_WIDGET(gtk_builder_get_object (builder, "frame-w-entry-0"));
    buttons.framewEntry[1] = GTK_WIDGET(gtk_builder_get_object (builder, "frame-w-entry-1"));
    buttons.framehEntry[0] = GTK_WIDGET(gtk_builder_get_object (builder, "frame-h-entry-0"));
    buttons.framehEntry[1] = GTK_WIDGET(gtk_builder_get_object (builder, "frame-h-entry-1"));
    buttons.offsetxEntry[0] = GTK_WIDGET(gtk_builder_get_object (builder, "offset-x-entry-0"));
    buttons.offsetxEntry[1] = GTK_WIDGET(gtk_builder_get_object (builder, "offset-x-entry-1"));
    buttons.offsetyEntry[0] = GTK_WIDGET(gtk_builder_get_object (builder, "offset-y-entry-0"));
    buttons.offsetyEntry[1] = GTK_WIDGET(gtk_builder_get_object (builder, "offset-y-entry-1"));
    buttons.suffixEntry = GTK_WIDGET(gtk_builder_get_object (builder, "suffix-entry"));

    g_signal_connect(G_OBJECT(buttons.recordBtn), "clicked", G_CALLBACK (button_record), &buttons);
    g_signal_connect(G_OBJECT(buttons.stopBtn),   "clicked", G_CALLBACK (button_stop), &buttons);
    g_signal_connect(G_OBJECT(buttons.resetBtn),  "clicked", G_CALLBACK (button_reset), &buttons);
    g_signal_connect(G_OBJECT(buttons.applyBtn),  "clicked", G_CALLBACK (button_apply), &buttons);
    g_signal_connect(G_OBJECT(buttons.acqStartBtn),  "clicked", G_CALLBACK (button_acq_start), &buttons);
    g_signal_connect(G_OBJECT(buttons.acqStopBtn),  "clicked", G_CALLBACK (button_acq_stop), &buttons);
    g_signal_connect(G_OBJECT(buttons.hMirror[0]), "toggled", G_CALLBACK(button_mirror_state), &buttons);
    g_signal_connect(G_OBJECT(buttons.wMirror[0]), "toggled", G_CALLBACK(button_mirror_state), &buttons);
    g_signal_connect(G_OBJECT(buttons.hMirror[1]), "toggled", G_CALLBACK(button_mirror_state), &buttons);
    g_signal_connect(G_OBJECT(buttons.wMirror[1]), "toggled", G_CALLBACK(button_mirror_state), &buttons);

    progress_bar = GTK_WIDGET(gtk_builder_get_object (builder, "progress-bar"));
    gtk_progress_bar_set_show_text (GTK_PROGRESS_BAR(progress_bar), TRUE);

    g_print("Initialising devices \n");
    if (!init_devices()) {
        camera_configure(&CamId0);
        camera_configure(&CamId1);
    } else {
        fake_cameras = TRUE;
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

    for (unsigned int k=0; k<2; k++)
        frame[k] = sframealloc(config.camhmax[k]/3, config.camwmax[k]/3, 3);

    for (unsigned int k=0; k<2; k++)
        frame_shadow[k] = sframealloc(config.camhmax[k]/3, config.camwmax[k]/3, 3);

    image[0] = GTK_IMAGE(gtk_builder_get_object(builder, "image-0"));
    image[1] = GTK_IMAGE(gtk_builder_get_object(builder, "image-1"));

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



    gtk_main();

    return 0;
}
