#ifndef _TEST_APP
#define _TEST_APP

#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxGui.h"
#include "ofMain.h"
#include "msbOFCore.h"
#include "actor.h"

#include "basicButton.h"
#include "textInputButton.h"
#include "assignButton.h"
#include "sliderButton.h"



// Uncomment this to use a camera instead of a video file
#define CAMERA_CONNECTED
//#define SCREENRESX 1920+1920+1280
//#define SCREENRESY 1080
#define SCREENRESX 1366
#define SCREENRESY 768
#define NMAXBLOBS 10
#define TRACKBUFFER 10

struct actorID;
struct memberID;

class testApp : public ofBaseApp, public Actor{

	public:
	    testApp();
	    virtual ~testApp();

		void setup();
		void update();
		void draw();

		//void interfaceSetup();

		//void cornerSetup();
		//void connectSetup();

		//from msbOFCore
		//void msbSetup();
		//void registerProperties();
		//void trigger(Actor* other);
		//void checkConnections(Actor* other);
        //void loadSettings();

        void trackPoints();
        void clearTrackPointBuffer();
        void calculateWhoHitTheWall();
        void calculateWhoHitTheWallCostant(int & placeholder);

        void applyMask();
        //void accumulateImage();
        //void accumulateMask();
        void postProcessMask();

       // void drawGrid(int dimSquare);
       // void drawFill(int x, int y);
        // void drawConnect(int x1, int y1, int x2, int y2);

		//void findHomography(ofPoint src[4], ofPoint dst[4], float homography[16]);
		//void gaussian_elimination(float *input, int n);

		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);

        ofFbo rgbaFbo; // with alpha

        ofImage beginImage;
        ofImage baseImage;
        ofImage buildingImage;
        ofImage roadImage;

        ofImage stageImage;

        /*Vector4f xP;
        Vector4f yP;

        Vector4f xP2;
        Vector4f yP2;

        Vector4f xC;
        Vector4f yC;*/


        Matrix4f pMat1;
        Matrix4f pMat2;
        Matrix4f cMat;

        int mX;
        int mY;

        /*//Resolution second & third "window"
        int mainW;
        int mainH;
        int firstW;
        int firstH;
        int secondW;
        int secondH;*/

        int currentFrame;

        //Kinect stuff from here

        ofxKinect kinect;

        ofxCvGrayscaleImage ocvBufferedImage;
        ofxCvGrayscaleImage ocvImage;
        ofxCvGrayscaleImage ocvMask;
        ofxCvGrayscaleImage ocvDiff;
        ofxCvGrayscaleImage ocvDeepImage;

        ofxCvContourFinder contourFinder;
        /*int minDimBlob;
        int maxDimBlob;

        int minLimitTarget;
        int maxLimitTarget;*/

        //ofVideoPlayer   mockup;

        unsigned char*               pixelBufferOne;
        unsigned char*               pixelBufferTwo;
        unsigned char*               pixelBufferThree;

        //int threshold;
        float lineWidth;
        float rectSize;
        //float trackDistance;

        //int dilateAmount;
        //int erodeAmount;
        int blurAmount;

        int imageBuffer;

        int selectedPoint;

        bool bInvertMask;
        bool bDrawGrid;
        bool bAccumulateMask;
        bool bMockup;

        Vector3f trackPointsTemp[NMAXBLOBS];
        vector<Vector3f> trackPointBuffer[NMAXBLOBS];
        unsigned char trackPointBufferColor[NMAXBLOBS];
        bool trackPointActiveBlobs[NMAXBLOBS];
        Vector3f whoHitTheWall[NMAXBLOBS];
        int hitted[NMAXBLOBS];
        /*vector<BasicButton*>  connectors;
        vector<BasicButton*>  connected;*/


        ofTrueTypeFont valueNumber;
        char valueStr[255]; // an array of chars

        void refreshPostProcessMask(int & placeholder);

        float hitTheWallConstant;

        ofxIntSlider erodeAmount;
        ofxIntSlider dilateAmount;
        ofxIntSlider threshold;
        ofxIntSlider minDimBlob;
        ofxIntSlider maxDimBlob;
        ofxIntSlider minLimitTarget;
        ofxIntSlider maxLimitTarget;
        ofxIntSlider surfaceYpositionMin;
        ofxIntSlider surfaceYpositionMax;
        ofxFloatSlider trackDistance;

        ofxIntSlider hitTheWallMaxHeightY;
        ofxIntSlider hitTheWallMaxDeepColor;
        ofxIntSlider hitTheWallMaxDeepColorBase;

        ofxPanel gui;


        //SliderButton* slBut;
};

#endif
