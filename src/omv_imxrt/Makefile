# Sources
SRCS += $(addprefix ,   \
	main.c              \
	xalloc.c            \
	fb_alloc.c          \
	umm_malloc.c        \
	ff_wrapper.c        \
	ini.c               \
	framebuffer.c       \
	array.c             \
	usbdbg.c            \
	cambus.c            \
	ov9650.c            \
	ov5640.c \
	ov2640.c            \
	ov7725.c            \
	mt9v034.c           \
	lepton.c \
	sensor.c            \
	mutex.c             \
   )

SRCS += $(addprefix img/,   \
	blob.c                  \
	binary.c				\
	clahe.c					\
	qrcode.c                \
	apriltag.c              \
	dmtx.c                  \
	draw.c					\
	zbar.c                  \
	fmath.c                 \
	fsort.c                 \
	qsort.c					\
	fft.c                   \
	filter.c				\
	haar.c                  \
	imlib.c                 \
	collections.c           \
	stats.c                 \
	selective_search.c		\
	integral.c              \
	integral_mw.c           \
	kmeans.c                \
	mathop.c				\
	lab_tab.c               \
	line.c					\
	xyz_tab.c               \
	yuv_tab.c               \
	rainbow_tab.c           \
	rgb2rgb_tab.c           \
	invariant_tab.c         \
	pool.c                  \
	point.c                 \
	rectangle.c             \
	bmp.c                   \
	ppm.c                   \
	gif.c                   \
	mjpeg.c                 \
	jpeg.c					\
	fast.c                  \
	agast.c                 \
	orb.c                   \
	template.c              \
	phasecorrelation.c      \
	font.c                  \
	lbp.c                   \
	eye.c                   \
	hough.c                 \
	lsd.c                   \
	sincos_tab.c            \
	shadow_removal.c		\
	edge.c                  \
	hog.c                   \
   )
# 	morph.c                 \
	midpoint.c              \
	mean.c                  \
	mode.c                  \
	median.c                \
	jpeg.c                  \
	lenet.c                 \
	lenet_model_num.c       \
	
SRCS += $(addprefix py/, \
	py_omv.c				\
	py_helper.c             \
	py_sensor.c             \
	py_lcd.c            	\
	py_image.c              \
	py_time.c               \
	py_gif.c                \
	py_mjpeg.c              \
   )

#	py_fir.c                \
	py_winc.c               \
	py_cpufreq.c            \
	
	
SRCS += $(addprefix nn/, 	\
	nn.c    		        \
   )

OBJS = $(addprefix $(BUILD)/, $(SRCS:.c=.o))
OBJ_DIRS = $(sort $(dir $(OBJS)))

all: | $(OBJ_DIRS) $(OBJS)
$(OBJ_DIRS):
	$(MKDIR) -p $@

$(BUILD)/%.o : %.c
	$(ECHO) "CC $<"
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD)/%.o : %.s
	$(ECHO) "AS $<"
	$(AS) $(AFLAGS) $< -o $@
