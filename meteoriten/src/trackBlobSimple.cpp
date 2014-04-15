#include "trackBlobSimple.h"

TrackBlobSimple::TrackBlobSimple(){
    minAreaBlob = 20;
    maxAreaBlob = 20000000;
    nMaxBlobs = 10;
    nHistoryTrackbuffer = 12;
    bFindHoles = false;
    bUseApproximation = true;

    trackDistance=100;
}

void TrackBlobSimple::setup(ofxCvGrayscaleImage t_sourceImage,int t_minAreaBlob,int t_maxAreaBlob,int t_nMaxBlobs,int t_nHistoryTrackbuffer,
                                      bool t_bFindHoles, bool t_bUseApproximation){

    minAreaBlob = t_minAreaBlob;
    maxAreaBlob = t_maxAreaBlob;
    nMaxBlobs = t_nMaxBlobs;
    nHistoryTrackbuffer = t_nHistoryTrackbuffer;
    bFindHoles = t_bFindHoles;
    bUseApproximation = t_bUseApproximation;
    sourceImage = t_sourceImage;

    //set up tracking buffers
    //maximal number of ID/Blob
    //lenght of the tracking history for each blob
    vector<vector<ofVec3f> > tempColumVect(nMaxBlobs, vector<ofVec3f>(nHistoryTrackbuffer, ofVec3f(0,0,0)));

    for (int i=0;i<nMaxBlobs;i++){
        trackPoint.push_back(ofVec3f(0,0,0));
        //set up a buffer as big as BUFFER
        for (int b=0;b<nHistoryTrackbuffer;b++){// ???
            //trackPointBuffer[i].push_back(ofVec3f(0,0,0));
            //trackPointBuffer.push_back()
        }
    }
}

void TrackBlobSimple::update(){

    ofVec3f currentTrackPoint;

    contourFinder.findContours(sourceImage,minAreaBlob,maxAreaBlob,nMaxBlobs,bFindHoles,bUseApproximation);

    for(int i = 0; i < contourFinder.nBlobs; i++) {

        //find the lowest point of each Blob
        float maxim=0;  //maximum y
        int myMin=0;    //point that is lowest in y

        for (int p=0;p<contourFinder.blobs[i].nPts;p++){
            if (contourFinder.blobs[i].pts[p].y<maxim){
                myMin=p;
                maxim=contourFinder.blobs[i].pts[p].y;
            }
        }

        currentTrackPoint.x=contourFinder.blobs[i].pts[myMin].x;
        currentTrackPoint.y=contourFinder.blobs[i].pts[myMin].y;

        //if we have buffered values, delete the oldest one
        //now we must associate the currently tracked point with the closest point we already have
        //compare location, find closest already tracked point

        float shortestDist=trackDistance;

        int currentlyClosestBuffer=-1;  //set to -1, if it stays there, we probably have a new puppet to track!

        //the Blob search the closest point in the buffer
        //or the first empty buffer position? //TODO right?
        for (int l=0;l<nMaxBlobs;l++){
            float dist=(currentTrackPoint-trackPointBuffer[l][0]).length();//TODO why here a float declaration?
            //if we are close to a buffered point, use that one
            if (dist<shortestDist){
                currentlyClosestBuffer=l;
                shortestDist=dist;
            }
            //if we happen upon a previously unused buffer, and we have not made a connection with any of the other buffers
            //and thus will use this one from now!
            if (trackPointBuffer[l][0].length()==0 && currentlyClosestBuffer==-1){
                currentlyClosestBuffer=l;
                shortestDist=0;
                l=nMaxBlobs; //break out of for-loop
            }
        }

        //if we have found something...
        if (currentlyClosestBuffer>-1){
            //if we have buffered values, delete the oldest one
            //and insert the current Blob position into buffer
            if (trackPointBuffer[currentlyClosestBuffer].size()== nHistoryTrackbuffer)
                trackPointBuffer[currentlyClosestBuffer].pop_back();

            trackPointBuffer[currentlyClosestBuffer].insert(trackPointBuffer[currentlyClosestBuffer].begin(),currentTrackPoint);
        }
    }
}


//To make the tracking more fluid we take the average
//of the history for each tracked point from the trackPointBuffer
void TrackBlobSimple::averageHistory(){

    for (int i=0;i<nMaxBlobs;i++){
        for (int b=0;b<nHistoryTrackbuffer;b++){
            //trackPoint[i]+=trackPointBuffer[i][b]/float(TRACKBUFFER);//warum sum(vect_i/num_tot)?
            trackPoint[i]+=trackPointBuffer[i][b];
            //trackPoint[i] = trackPointBuffer[i][b];
        }
        trackPoint[i]=trackPoint[i]/float(nHistoryTrackbuffer);
    }
}


//we draw the "id" number. It follow the tracked Blob
//TODO: maybe too much iteration... or what made the tracking so slow?
void TrackBlobSimple::drawIDs(){

    for (int i=0;i<nMaxBlobs;i++){
        //ofCircle(trackPoint[i].x,trackPoint[i].y,10);
        char buf[5];
        sprintf(buf,"%d",i);
        ofDrawBitmapString(buf,trackPoint[i].x,trackPoint[i].y,25);
    }
}

void TrackBlobSimple::drawCountornFinder(){

    contourFinder.draw();
}

void TrackBlobSimple::resetBuffer(){

    cout << "resetting trackPoint Buffer" << endl;
    //for each Puppet potentially to be tracked
    for (int i=0;i<nMaxBlobs;i++){
        trackPointBuffer[i].clear();
        //set up a buffer as big as TRACKBUFFER
        for (int b=0;b<nHistoryTrackbuffer;b++){
            trackPointBuffer[i].push_back(ofVec3f(0,0,0));
        }
    }

}


