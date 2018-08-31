#include "jniAlgo.h"

#include "ninebot_log.h"

using namespace ninebot_algo;

const jboolean PACKAGE_IS_SERVICE = true;
jint JNI_OnLoad(JavaVM* aVm, void* aReserved)
{
	JNIEnv* env;
	ALOGD ("JNI_OnLoad");

	if (aVm->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK)
	{
		ALOGE ("Failed to get JNI_1_6 environment");
		return -1;
	}

	if (init_algo(env)) {
		return -1;
	}

	ALOGD ("JNI_OnLoad end");
	return JNI_VERSION_1_6;
}
