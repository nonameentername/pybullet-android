LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libLinearMath
LOCAL_SRC_FILES := ../../bullet-android/build/bullet/bullet/src/LinearMath/libLinearMath.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libBulletCollision
LOCAL_SRC_FILES := ../../bullet-android/build/bullet/bullet/src/BulletCollision/libBulletCollision.a
include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := libBulletDynamics
LOCAL_SRC_FILES := ../../bullet-android/build/bullet/bullet/src/BulletDynamics/libBulletDynamics.a
include $(PREBUILT_STATIC_LIBRARY)


include $(CLEAR_VARS)

LOCAL_MODULE := bullet
LOCAL_MODULE_FILENAME := bullet
LOCAL_SRC_FILES := bullet.cpp
LOCAL_C_INCLUDES := $(LOCAL_PATH)/../../bullet-android/bullet/bullet/src/
LOCAL_STATIC_LIBRARIES := libBulletDynamics libBulletCollision libLinearMath
LOCAL_LDLIBS := -llog -ldl
LOCAL_LDLIBS := $(LOCAL_LDLIBS) $(call host-path,$(NDK_ROOT)/sources/cxx-stl/gnu-libstdc++/libs/$(TARGET_ARCH_ABI)/libsupc++.a) 

include $(BUILD_SHARED_LIBRARY)
