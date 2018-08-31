/*! define the base class for Ninebot algorithm
 * need to derive class AlgoXX and implement
 * 
 * Filename: AlgoBase.h
 * Version: 0.60
 * This file is common to many different projects, DO NOT CHANGE THIS FILE unless discussed with Zichong Chen
 * Each modification shall come with a version number update
 * Algo team, Ninebot Inc., 2016
 */

#ifndef NINEBOT_ALGO_BASE_H
#define NINEBOT_ALGO_BASE_H

#include "RawData.h"
#include <stdint.h>
#include <mutex>
#include <vector>
#include <chrono>
#include <thread>

#include "ninebot_log.h"

namespace ninebot_algo
{

//#ifndef WIN32
//#include <android/log.h>
//#define LOG_TAG		"AprAlgo"
//#define ALOGI(...) __android_log_print (ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
//#define ALOGD(...) __android_log_print (ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
//#define ALOGE(...) __android_log_print (ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
//#define ALOGW(...) __android_log_print (ANDROID_LOG_WARN, LOG_TAG, __VA_ARGS__)
//#else
//#define ALOGI(...) printf(__VA_ARGS__); printf("\n")
//#define ALOGD(...) printf(__VA_ARGS__); printf("\n")
//#define ALOGE(...) printf(__VA_ARGS__); printf("\n")
//#define ALOGW(...) printf(__VA_ARGS__); printf("\n")
//#endif

#define BucketSize 20	/*! define the maximum elements that input and output container can support */

/*! class of output containter for AlgoBase, the container includes an array of float values, a single timestamps represent the time that all output is generated */
class  OutputBucket
{
public:
	OutputBucket(){
		std::fill_n(vbuf, BucketSize, 0.0f);
		timestamp = 0;
	}
	~OutputBucket(){};

	/*! Set the ind-th output in the format of float value */
	void setValue(int ind, float val)
	{
		std::lock_guard<std::mutex> lock(mMutexVal);
		vbuf[ind] = val;
	}
	/*! Set the timestamp of all current outputs */
	void setTime(int64_t ts){
		std::lock_guard<std::mutex> lock(mMutexVal);
		timestamp = ts;
	}
	/*! Get the ind-th output float value */
	float getValue(int ind){
		std::lock_guard<std::mutex> lock(mMutexVal);
		return vbuf[ind];
	}
	/*! Get the timestamp of all current outputs */
	int64_t getTime(){
		std::lock_guard<std::mutex> lock(mMutexVal);
		return timestamp;
	}

private:
	float vbuf[BucketSize]; // init to 0
	int64_t timestamp; // init to 0
	std::mutex mMutexVal;
};

/*! class of input containter for AlgoBase, the container includes an array of float values, int64_t timestamps,
 * and bool that indicates whether the value is updated or not. Max number of BucketSize can be support.
 */
class InputBucket
{
public:
	/*! Constructor of InputBucket
	 * @param obArray The pointer to the OutputBucket array allocated in main(), each element is a dedicated output container for an algorithm instance
	 * @param ind The multiplexer to wire InputBucket of current algorithm instance to the OutputBucket array allocated in main()
	*/
	InputBucket(OutputBucket* obArray, std::vector<std::pair<int,int>>& ind){
		m_obArray = obArray;
		m_ind = ind;
		for(int i=0;i<BucketSize;i++){
			vbuf[i] = 0;
			tbuf[i] = 0;
			isUpdated[i] = false;
		}
	}
	~InputBucket(){};

	/*! Refresh this input container from values stored in OutputBucket array. Float value, timestamp, and update indicator is all updated.
	 * The update indicator is set to true if the timestamp of a float value is different from last time
	*/
	void getInput()
	{
		std::lock_guard<std::mutex> lock(mMutexVal);
		for(int i=0; i<m_ind.size();i++){
			vbuf[i] = m_obArray[m_ind[i].first].getValue(m_ind[i].second);
			int64_t t = m_obArray[m_ind[i].first].getTime();
			if(t == tbuf[i])
				isUpdated[i] = false;
			else{
				tbuf[i] = t;
				isUpdated[i] = true;
			}
		}
	}
	/*! After call getInput(), return ind-th float value */
	float getValue(int ind){
		std::lock_guard<std::mutex> lock(mMutexVal);
		return vbuf[ind];
	}
	/*! After call getInput(), return ind-th timestamp */
	int64_t getTime(int ind){
		std::lock_guard<std::mutex> lock(mMutexVal);
		return tbuf[ind];
	}
	/*! After call getInput(), return ind-th update indicator */
	bool getIsUpdated(int ind){
		std::lock_guard<std::mutex> lock(mMutexVal);
		return isUpdated[ind];
	}

private:
	float vbuf[BucketSize]; // init to 0
	int64_t tbuf[BucketSize]; //init to 0
	bool isUpdated[BucketSize]; // init to false
	OutputBucket* m_obArray;
	std::vector<std::pair<int,int>> m_ind;
	std::mutex mMutexVal;
};

/*! Base class for all Ninebot algorithm */
class AlgoBase {
public:
	/*! Constructor of AlgoBase
	 * @param rawInterface The pointer to the RawData object instantiated in main()
	 * @param run_sleep_ms The sleep time in millisecond for AlgoBase::run() - AlgoBase::step() is executed following a sleep in run()
	 * @param use_imu Enable this to dispatch IMU data to the Algo
	*/
	AlgoBase(RawData* rawInterface, int run_sleep_ms, bool use_imu = false){
		mRawDataInterface = rawInterface;
		mUseIMU = use_imu;
		if(mUseIMU)
			mRawDataInterface->registerIMUCallback((AlgoBase*)this);
		m_run_sleep_ms = run_sleep_ms;
		IsRunning = false;
		mbStopped = false;
		mbStopRequested = false;
	}

	/*
	 * 	 * @param obArray The pointer to the OutputBucket array allocated in main(), each element is a dedicated output container for an algorithm instance
	 * @param ind The multiplexer to wire InputBucket of this algorithm instance to the OutputBucket array allocated in main()
	 * @param id The index number of obArray that specifies the output container of this algorithm instance

	 AlgoBase(RawData* rawInterface, OutputBucket* obArray, std::vector<std::pair<int,int>> ind, int id, int run_sleep_ms, bool use_imu = false){
		mRawDataInterface = rawInterface;
		mUseIMU = use_imu;
		if(mUseIMU)
			mRawDataInterface->registerIMUCallback((AlgoBase*)this);
		m_run_sleep_ms = run_sleep_ms;
		mInputBucket = NULL;
		mOutputBucket = &(obArray[id]);
		mInputBucket = new InputBucket(obArray, ind);
		IsRunning = false;
		mbStopped = false;
		mbStopRequested = false;
	}*/

	 ~AlgoBase(){
	 	exit();
		if(mUseIMU)
		mRawDataInterface->deregisterIMUCallback();
		//if(mInputBucket)
		//	delete mInputBucket;
	}

	/*! Run algorithm once, this is a virtual function, shall be implemented in derived class */
	virtual bool step() = 0;
	/*! IMU callback if needed */
	virtual void UpdateIMU(StampedIMU val){};
	virtual void UpdateAccel(MTPoint val){};
	virtual void UpdateGyro(MTPoint val){};

	/*! Start run() thread */
	bool start()
	{
		if(!IsRunning){
            IsRunning = true;
            mAlgoThread = thread(&AlgoBase::run,this);
            return true;
        } else
            return false;

	}

	/*! Exit run() thread */
	bool exit()
	{
		if(IsRunning){
			release();
		    IsRunning = false;
		    mAlgoThread.join();
		    return true;
		}
		else
		    return false;


	}

	/*! Pause run() thread */
	void pause()
	{
		requestStop();
		while( !isStopped())
			this_thread::sleep_for(chrono::milliseconds(100));
	}

	/*! Resume run() thread */
	void resume()
	{
		release();
	}

protected:
	void run()
	{
#ifndef WIN32
		//ALOGD("AlgoCA thread id: %d\n", gettid());
#endif
		mRawDataInterface->attachJVM();
		//ALOGD("algo base attach jvm, pid = %d", pthread_self());
		while(IsRunning)
		{
		    auto start = std::chrono::system_clock::now();
			// single step of algorithm
			bool isExe = step();
			// Safe area to stop
			safeStop();
            // sleep for designated time
			auto end = std::chrono::system_clock::now();
            std::chrono::duration<double> diff = end-start;
			double deltat = 0;
			if(isExe)
             	deltat = m_run_sleep_ms-diff.count()*1000;
			else
				deltat = 5-diff.count()*1000;
            if(deltat>0)
                this_thread::sleep_for(chrono::milliseconds((int)deltat));
		}
		mRawDataInterface->detachJVM();
	}
	void requestStop()
	{
		std::lock_guard<std::mutex> lock(mMutexStop);
		mbStopRequested = true;
	}
	void stop()
	{
		std::lock_guard<std::mutex> lock(mMutexStop);
		mbStopped = true;
	}
	bool isStopped()
	{
		std::lock_guard<std::mutex> lock(mMutexStop);
		return mbStopped;
	}
	bool stopRequested()
	{
		std::lock_guard<std::mutex> lock(mMutexStop);
		return mbStopRequested;
	}
	void release()
	{
		std::lock_guard<std::mutex> lock(mMutexStop);
		if(mbStopped){
			mbStopped = false;
			mbStopRequested = false;
		}
	}
	void safeStop()
	{
		if(stopRequested())
		{
			stop();
			while(isStopped())
			{
				this_thread::sleep_for(chrono::milliseconds(100));
			}
		}
	}

	// variables
	//InputBucket* mInputBucket;
	//OutputBucket* mOutputBucket;
	RawData* mRawDataInterface;
	bool IsRunning;
	bool mbStopped;
	bool mbStopRequested;
	bool mUseIMU;
	int m_run_sleep_ms;
	std::mutex mMutexStop;
	thread mAlgoThread;
	
};

} // namespace ninebot_algo

#endif