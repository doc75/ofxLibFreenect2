#include "ofxLibfreenect2.h"

// TODO:
//   - disable rgb or depth stream
//--------------------------------------------------------------------------------

// static content
ofxLibfreenect2Context ofxLibfreenect2::_lf2Context;

ofxLibfreenect2::ofxLibfreenect2(){
    bNewFrame       = false;
    bNewBuffer      = false;
    bGrabberInited  = false;
    bIsConnected    = false;
    bUseTexture     = true;
    _bUseVideo      = true;
    _bUseDepth      = true;
    lastFrameNo     = -1;

    //set default distance range to 50cm - 600cm
    minDistance.set("minDistance", 500, 0, 12000);
    maxDistance.set("maxDistance", 6000, 0, 12000);
}

//--------------------------------------------------------------------------------
ofxLibfreenect2::~ofxLibfreenect2(){
    close();
}

//--------------------------------------------------------------------
bool ofxLibfreenect2::setup(int w, int h)
{
    return true;
}

//--------------------------------------------------------------------
bool ofxLibfreenect2::init(bool texture, bool video, bool depth) {
	if(isConnected()) {
		ofLogWarning("ofxLibfreenect2") << "init(): do not call init while ofxLibfreenect2 is running!";
		return false;
	}

    if( !depth && !video ) {
        ofLogWarning("ofxLibfreenect2") << "init(): Disabling both depth and rgb streams is not possible!" << std::endl;
        return false;
    } 

	//clear();
    bGrabberInited = false;

	bUseTexture = texture;
    _bUseVideo = video;
    _bUseDepth = depth;

	videoPixels.allocate(width, height, GL_RGBA);
	videoPixelsFront.allocate(width, height, GL_RGBA);
	videoPixelsBack.allocate(width, height, GL_RGBA);

	depthPixels.allocate(depthWidth, depthHeight, 1);
	depthPixelsFront.allocate(depthWidth, depthHeight, 1);
	depthPixelsBack.allocate(depthWidth, depthHeight, 1);

	videoPixels.set(0);
	videoPixelsFront.set(0);
	videoPixelsBack.set(0);

	depthPixels.set(0);
	depthPixelsFront.set(0);
	depthPixelsBack.set(0);

	rawDepthPixels.set(0);

	if(bUseTexture) {
		depthTex.allocate(depthWidth, depthHeight, GL_LUMINANCE);
		videoTex.allocate(width, height, GL_RGB);
	}


	bGrabberInited = true;

	return bGrabberInited;
}

//--------------------------------------------------------------------------------
bool ofxLibfreenect2::isInitialized() const
{
    return bGrabberInited;
}

//---------------------------------------------------------------------------
bool ofxLibfreenect2::isConnected() const{
	return isThreadRunning();
}

//--------------------------------------------------------------------------------
bool ofxLibfreenect2::isConnected(int id)
{
    return isConnected();
}

//--------------------------------------------------------------------------------
bool ofxLibfreenect2::open(){
    return open(-1);
}

bool ofxLibfreenect2::open(int iIndex) {
    std::string serial = _lf2Context.getDeviceSerialNumber(iIndex);
    return open(serial);
}

bool ofxLibfreenect2::open(std::string iSerial) {
    close();
    serial = iSerial;

    bool ret = _lf2Context.open(*this, serial);

    if(ret)
    {
        ofLogNotice("ofxLibfreenect2") << "Opening device with serial: " << dev->getSerialNumber() << " device firmware: " << dev->getFirmwareVersion();
        lastFrameNo = -1;
        startThread();
    } else {
        ofLogError("ofxLibfreenect2") << "Failure opening device with serial = " << serial << " !" << std::endl;
    }

    bIsConnected = true;

    return ret;
}



void ofxLibfreenect2::listDevices() {
    if(!isInitialized()) {
        init();
    }

    int numDevices = _lf2Context.enumerateDevices();

	if(numDevices == 0) {
		ofLogNotice("ofxLibfreenect2") << "no devices found";
	}
	else if(numDevices == 1) {
		ofLogNotice("ofxLibfreenect2") << 1 << " device found";
	}
	else {
		ofLogNotice("ofxLibfreenect2") << numDevices <<" devices found";
	}

}

//--------------------------------------------------------------------------------
void ofxLibfreenect2::threadedFunction(){
    //listener = new listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    int types = 0;
    if( _bUseVideo ) {
        types |= libfreenect2::Frame::Color;
    }
    if( _bUseDepth ) {
        types |= libfreenect2::Frame::Depth;
    } 
    libfreenect2::SyncMultiFrameListener listener(types);
    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

    
    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->startStreams(_bUseVideo, _bUseDepth);

    registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

    while(isThreadRunning()){
		listener.waitForNewFrame(frames);
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		//libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        if( _bUseVideo && _bUseDepth ) {
		    registration->apply(rgb,depth,&undistorted,&registered);
        }

        lock();


        if( _bUseVideo ) {
            videoPixelsBack.setFromPixels(rgb->data, rgb->width, rgb->height, OF_PIXELS_RGBA);
            videoPixelsFront.swap(videoPixelsBack);
        }
        if( _bUseDepth ) {
            depthPixelsBack.setFromPixels((float *)depth->data, depth->width, depth->height, 1);
            depthPixelsFront.swap(depthPixelsBack);
        }


  //      lock();
        bNewBuffer = true;
        unlock();

        //ofSleepMillis(2);

		listener.release(frames);
    }

    dev->stop();
    dev->close();
    dev = 0;

    delete registration;
    registration = 0;
}

//--------------------------------------------------------------------------------
void ofxLibfreenect2::update(){
    if(!bGrabberInited) return;
    if(!isConnected()) return;

    if( ofGetFrameNum() != (unsigned int) lastFrameNo ){
        bNewFrame = false;
        lastFrameNo = ofGetFrameNum();
    }
    if( bNewBuffer ){
        lock();
            //videoPixelsFront.swapRgb();
            videoPixels = videoPixelsFront;
            videoPixels.swapRgb();
            rawDepthPixels = depthPixelsFront;
            bNewBuffer = false;
        unlock();

        if( rawDepthPixels.size() > 0 ){
            if( depthPixels.getWidth() != rawDepthPixels.getWidth() ){
                depthPixels.allocate(rawDepthPixels.getWidth(), rawDepthPixels.getHeight(), 1);
            }

            //float * pixelsF         = rawDepthPixels.getData();
            unsigned char * pixels  = depthPixels.getData();

            for(int i = 0; i < depthPixels.size(); i++){
                pixels[i] = ofMap(rawDepthPixels[i], minDistance, maxDistance, 255, 0, true);
                if( pixels[i] == 255 ){
                    pixels[i] = 0;
                }
            }

        }

        depthTex.loadData( depthPixels, GL_LUMINANCE );
        videoTex.loadData( videoPixels, GL_RGBA );

        bNewFrame = true;
    }
}

//--------------------------------------------------------------------------------
bool ofxLibfreenect2::isFrameNew() const {
    return bNewFrame;
}

//------------------------------------
bool ofxLibfreenect2::isUsingTexture() const{
	return bUseTexture;
}

//----------------------------------------------------------
void ofxLibfreenect2::draw(float _x, float _y, float _w, float _h) const{
	if(bUseTexture) {
		videoTex.draw(_x, _y, _w, _h);
	}
}

//----------------------------------------------------------
void ofxLibfreenect2::draw(float _x, float _y) const{
	draw(_x, _y, (float)width, (float)height);
}

//----------------------------------------------------------
void ofxLibfreenect2::drawDepth(float _x, float _y, float _w, float _h) const{
	if(bUseTexture) {
		depthTex.draw(_x, _y, _w, _h);
	}
}

//---------------------------------------------------------------------------
void ofxLibfreenect2::drawDepth(float _x, float _y) const{
	drawDepth(_x, _y, (float)width, (float)height);
}

//--------------------------------------------------------------------
bool ofxLibfreenect2::setPixelFormat(ofPixelFormat pixelFormat){
	if(pixelFormat==OF_PIXELS_RGB){
		return true;
	}else{
		return false;
	}
}

//--------------------------------------------------------------------
ofPixelFormat ofxLibfreenect2::getPixelFormat() const{
    return OF_PIXELS_RGB;
}

//----------------------------------------------------------
float ofxLibfreenect2::getHeight() const{
	return (float) height;
}

//---------------------------------------------------------------------------
float ofxLibfreenect2::getWidth() const{
	return (float) width;
}

//--------------------------------------------------------------------------------
ofPixels & ofxLibfreenect2::getDepthPixels(){
    return depthPixels;
}

//--------------------------------------------------------------------------------
const ofPixels & ofxLibfreenect2::getDepthPixels() const{
	return depthPixels;
}

//--------------------------------------------------------------------------------
ofPixels & ofxLibfreenect2::getPixels(){
	return videoPixels;
}

//--------------------------------------------------------------------------------
const ofPixels & ofxLibfreenect2::getPixels() const{
	return videoPixels;
}

//------------------------------------
ofFloatPixels & ofxLibfreenect2::getDistancePixels(){
	return rawDepthPixels;
}

//------------------------------------
const ofFloatPixels & ofxLibfreenect2::getDistancePixels() const{
	return rawDepthPixels;
}

//------------------------------------
ofTexture& ofxLibfreenect2::getTexture(){
	if(!videoTex.isAllocated()){
		ofLogWarning("ofxLibfreenect2") << "getTexture(): device " << deviceId << " video texture not allocated";
	}
	return videoTex;
}

//---------------------------------------------------------------------------
ofTexture& ofxLibfreenect2::getDepthTexture(){
	if(!depthTex.isAllocated()){
		ofLogWarning("ofxLibfreenect2") << "getDepthTexture(): device " << deviceId << " depth texture not allocated";
	}
	return depthTex;
}

//------------------------------------
const ofTexture& ofxLibfreenect2::getTexture() const{
	if(!videoTex.isAllocated()){
		ofLogWarning("ofxLibfreenect2") << "getTexture(): device " << deviceId << " video texture not allocated";
	}
	return videoTex;
}

//---------------------------------------------------------------------------
const ofTexture& ofxLibfreenect2::getDepthTexture() const{
	if(!depthTex.isAllocated()){
		ofLogWarning("ofxLibfreenect2") << "getDepthTexture(): device " << deviceId << " depth texture not allocated";
	}
	return depthTex;
}

//--------------------------------------------------------------------------------
void ofxLibfreenect2::close()
{
    if(bIsConnected) {
		stopThread();
		ofSleepMillis(10);
		waitForThread(false);
    }

    bNewFrame       = false;
    bNewBuffer      = false;
    bIsConnected    = false;
    lastFrameNo     = -1;

}

void ofxLibfreenect2::setDevice(libfreenect2::Freenect2Device * iDevice) {
    dev = iDevice;
}

void ofxLibfreenect2::setDeviceId(int iId) {
    deviceId = iId;
}

static bool sortKinectPairs(ofxLibfreenect2Context::freenect2Pair A, ofxLibfreenect2Context::freenect2Pair B){
    return A.serial < B.serial;
}

ofxLibfreenect2Context::ofxLibfreenect2Context() {

    _bInited = false;
    _pipeline = 0;
}

ofxLibfreenect2Context::~ofxLibfreenect2Context() {

    _bInited = false;
    _deviceList.clear();
    _pipeline = 0;

}

void ofxLibfreenect2Context::init() {

    if( _bInited ) {
        return;
    }

    _pipeline = new libfreenect2::OpenCLPacketPipeline();

    int nbDevices = enumerateDevices();
    for( int i = 0 ; i < nbDevices ; i++ ) {
        freenect2Pair kp2;
        kp2.id = i;
        kp2.serial = _freenect2.getDeviceSerialNumber(i);
        kp2.kinect = 0;
        kp2.connected = false;
        _deviceList.push_back(kp2);
    }

    // sort devices by serial number
    sort(_deviceList.begin(), _deviceList.end(), sortKinectPairs);
    _bInited = true;
}

bool ofxLibfreenect2Context::isInited() {
    return _bInited;
}

int ofxLibfreenect2Context::enumerateDevices() {
    return _freenect2.enumerateDevices();
}

std::string ofxLibfreenect2Context::getDeviceSerialNumber(int iIndex) {
    if( !isInited() ) {
        init();
    }

    std::string serial("");
    if( iIndex < 0 ) {
        // let's find the first kinect not connected
        for( int i = 0 ; i < _deviceList.size() ; i++ ) {
            if( !_deviceList[i].connected) {
                serial = _deviceList[i].serial;
                ofLogWarning("ofxLibfreenect2") << "getDeviceSerialNumber will return: " << serial;
                break;
            }
        }
    }
    else {
        serial = _freenect2.getDeviceSerialNumber(iIndex);
    }

    return serial;
}

bool ofxLibfreenect2Context::open(ofxLibfreenect2& iLibfreenect2, std::string iSerial) {
    if( !isInited() ) {
        init();
    }

    // we first check that device is not aready opened
    bool ret = true;
    bool alreadyOpened = false;
    int idx;
    for( idx = _deviceList.size() - 1 ; idx >= 0 ; idx-- ) {
        if( _deviceList[idx].serial == iSerial ) {
            if( _deviceList[idx].connected ) {
                alreadyOpened = true;
            }
            break;
        }
    }

    if( alreadyOpened ) {
        ofLogWarning("ofxLibfreenect2") << "open(ofxLibfreenect2&, std::string): kinect already opened!";
        ret = false;
    }
    else {
        libfreenect2::Freenect2Device * kinect = _freenect2.openDevice(iSerial, _pipeline);
        if( kinect ) {
          iLibfreenect2.setDevice(kinect);
          iLibfreenect2.setDeviceId(_deviceList[idx].id);
          _deviceList[idx].connected = true;
          _deviceList[idx].kinect = &iLibfreenect2;
        }
        else {
            ofLogWarning("ofxLibfreenect2") << "open(ofxLibfreenect2&, std::string): kinect returned by openDevice is NULL!";
            ret = false;
        }
    }
    return ret;
}

void ofxLibfreenect2Context::close(ofxLibfreenect2& kinect) {
    if( isInited() ) {
        bool found = false;
        for( int idx = _deviceList.size() - 1 ; idx >= 0 ; idx-- ) {
            if( _deviceList[idx].kinect == &kinect && _deviceList[idx].connected == true) {
                _deviceList[idx].connected = false;
                _deviceList[idx].kinect = 0;
                found = true;
                break;
            }
        }
        if( found ) {
            kinect.close();
        }
    }
}


