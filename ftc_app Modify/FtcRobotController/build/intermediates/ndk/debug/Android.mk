LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE := FtcRobotController
LOCAL_SRC_FILES := \
	C:\Users\Andrew\Documents\Robotics\ftc_app Modify\FtcRobotController\src\main\jni\TestClass.cpp \

LOCAL_C_INCLUDES += C:\Users\Andrew\Documents\Robotics\ftc_app Modify\FtcRobotController\src\main\jni
LOCAL_C_INCLUDES += C:\Users\Andrew\Documents\Robotics\ftc_app Modify\FtcRobotController\src\debug\jni

include $(BUILD_SHARED_LIBRARY)
