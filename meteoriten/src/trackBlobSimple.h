#ifndef _TRACK_BLOB_S
#define _TRACK_BLOB_S

#include <vector>
#include "ofxOpenCv.h"

class TrackBlobSimple{

  private:
    ofxCvContourFinder contourFinder;
    ofxCvGrayscaleImage sourceImage;

    vector<ofVec3f> trackPoint;
    vector< vector<ofVec3f> > trackPointBuffer;

    int nMaxBlobs;
    int nHistoryTrackbuffer;
    bool bFindHoles;
    bool bUseApproximation;

  public:
    TrackBlobSimple();

    int minAreaBlob;
    int maxAreaBlob;

    int trackDistance;

    void setup(ofxCvGrayscaleImage, int, int, int, int, bool, bool);
    void update();
    void averageHistory();
    void drawIDs();
    void drawCountornFinder();
    void resetBuffer();
};

#endif
