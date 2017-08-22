
#pragma once

#include "ofMain.h"
#include "ofxBase3DVideo.h"

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

class ofxLibfreenect2Context;

class ofxLibfreenect2 : public ofThread, public ofxBase3DVideo {

    public:
        ofxLibfreenect2();
        ~ofxLibfreenect2();

        // WARNING, the boolean and their order are not the same as in ofxKinect
        // this order is kept for compatibility with first version of
        // ofxLibfreenect2 which was managing only texture
        bool init(bool texture=true, bool video=true, bool depth=true);
        bool setup(int w, int h);
        // opens the first available device not yet opened
        // order of device is based on serial number (compliant with ofxKinect behavior)
        bool open();
        // opens the device with given index
        bool open(int iIndex);
        // opens the device with given serial number
        bool open(std::string iSerial);
        void update();
        void close();
        void listDevices();

        bool isInitialized() const;
        bool isConnected() const;
        // same as isConnected
        bool isConnected(int id);
        bool isFrameNew() const;

        ofPixels & getPixels();
        const ofPixels & getPixels() const;

        ofPixels& getDepthPixels();
        const ofPixels& getDepthPixels() const;

        /// (not implemented) get the distance in millimeters to a given point as a float array
        ofFloatPixels & getDistancePixels();
        const ofFloatPixels & getDistancePixels() const;

        /// get the video (ir or rgb) texture
        ofTexture& getTexture();
        const ofTexture& getTexture() const;

        ofTexture& getDepthTexture();
        const ofTexture& getDepthTexture() const;

        bool setPixelFormat(ofPixelFormat pixelFormat);
        ofPixelFormat getPixelFormat() const;

        void setUseTexture(bool bUse);
        bool isUsingTexture() const;

        void draw(float x, float y, float w, float h) const;
        void draw(float x, float y) const;

        void drawDepth(float x, float y, float w, float h) const;
        void drawDepth(float x, float y) const;

        float getHeight() const;
        float getWidth() const;

        ofParameter <float> minDistance;
        ofParameter <float> maxDistance;

        const static int width = 1920;
        const static int height = 1080;
        const static int depthWidth = 512;
        const static int depthHeight = 424;

        void setDevice(libfreenect2::Freenect2Device * iDevice);
        void setDeviceId(int iId);

    protected:
        void threadedFunction();

        ofPixels videoPixelsFront;
        ofPixels videoPixelsBack;
        ofPixels videoPixels;
        ofFloatPixels depthPixelsFront;
        ofFloatPixels depthPixelsBack;
        ofPixels depthPixels;
        ofFloatPixels rawDepthPixels;
        ofPixelFormat pixelFormat;

        bool bNewBuffer;
        bool bNewFrame;
        bool bGrabberInited;
        bool bIsConnected;
        bool bUseTexture;
        bool _bUseDepth;
        bool _bUseVideo;



        ofTexture depthTex;
        ofTexture videoTex;

        int lastFrameNo;

        int deviceId;

		libfreenect2::Freenect2Device *dev = 0;
		libfreenect2::Registration* registration;
		libfreenect2::SyncMultiFrameListener* listener = 0;
		libfreenect2::FrameMap frames;

		string serial;
        static ofxLibfreenect2Context _lf2Context;

};

//======================================================
// Do not use it directly
// should only be used by ofxLibfreenect2 class
//======================================================
class ofxLibfreenect2Context {
  
    public:

        ofxLibfreenect2Context();
        ~ofxLibfreenect2Context();

        // return the number of devices detected
        int enumerateDevices();

        // return the serial number of kinect at given index
        std::string getDeviceSerialNumber(int iIndex = -1);
        // open the kinect having the given serial number
        bool open(ofxLibfreenect2& iKinect, std::string iSerial);

        // close the kinect
        void close(ofxLibfreenect2& iKinect);

        struct freenect2Pair {
            std::string serial;
            int id;
            ofxLibfreenect2* kinect;
            bool connected;
        };
    protected:
        void init();
        bool isInited();

    private:
        bool _bInited;
        libfreenect2::Freenect2 _freenect2;
        libfreenect2::PacketPipeline * _pipeline;
        std::vector<freenect2Pair> _deviceList;

};

