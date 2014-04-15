#include "testApp.h"

//--------------------------------------------------------------

testApp::testApp(){}

testApp::~testApp(){

}

//=======================================SETUP FUNCTIONS
void testApp::setup(){

    //ofSetFrameRate(25);

    currentFrame=0;

    mX=0;
    mY=0;

    threshold=40;
    //trackDistance=100;
    imageBuffer=8;

    lineWidth=4.0;

    erodeAmount=0;
    dilateAmount=0;
    blurAmount=0;

    selectedPoint=0;

    bAccumulateMask=false;
    bInvertMask=true;
    bDrawGrid=true;

    ocvBufferedImage.allocate(640,480);
    ocvImage.allocate(640,480);
    ocvMask.allocate(640,480);
    ocvDiff.allocate(640,480);

    ocvDiff.set(255);
    ocvMask.set(255);

    kinect.bImage = true;
    kinect.init();
    kinect.setVerbose(true);
    kinect.open();
    kinect.cutOffFar=8192;

	ofBackground(0,0,64);

    pixelBufferOne= new unsigned char[640*480];
    pixelBufferTwo= new unsigned char[640*480];
    pixelBufferThree= new unsigned char[640*480];

    /*//set up tracking buffers
    //for each Puppet potentially to be tracked
    for (int i=0;i<NMAXBLOBS;i++){
        //set up a buffer as big as BUFFER
        for (int b=0;b<TRACKBUFFER;b++){// ???
            trackPointBuffer [i].push_back(Vector3f(0,0,0));
        }
    }*/

    minDimBlob = 100;
    maxDimBlob = 200000;

    trackingBlobs.setup(ocvImage,minDimBlob,maxDimBlob,NMAXBLOBS,TRACKBUFFER,false,true);

    //ofSetFrameRate(25);


    ofDisableArbTex();

    rgbaFbo.allocate(1280, 800, GL_RGBA); // with alpha, 8 bits red, 8 bits green, 8 bits blue, 8 bits alpha, from 0 to 255 in 256 steps

    rgbaFbo.begin();

        ofClear(255,255,255, 0);

    rgbaFbo.end();

    valueNumber.loadFont("verdana.ttf", 25);


    //stage simulation
    /*stageImage.loadImage("stage.tga");
    mockup.loadMovie("mockup.mp4");*/

    //stageImage.getTextureReference().texData.textureTarget=GL_TEXTURE_2D;

}

//======================================UPDATE FUNCTIONS
void testApp::update(){

    rectSize= lineWidth *64;

    currentFrame++;
    kinect.update();


    /*if (bAccumulateMask)
        accumulateMask();*/

    //buffer image
    //if(currentFrame%imageBuffer==0){
        //reset image to be accumulated
        //ocvImage.setFromPixels(ocvBufferedImage.getPixels(),640,480);
        //ocvBufferedImage.setFromPixels(kinect.depthPixels,640,480);
    //}
    ocvImage.setFromPixels(kinect.depthPixels,640,480);

    //do all the stuff all the time
    applyMask();
    ocvImage.threshold(threshold,false); //TODO for blob detection is better grayscale or B/W?
    //trackPoints();
    trackingBlobs.update();

}

/*void testApp::trackPoints(){

    Vector3f trackPoint;

    contourFinder.findContours(ocvImage,minDimBlob,maxDimBlob,NMAXBLOBS,false,true);

    for(int i = 0; i < contourFinder.nBlobs; i++) {
        //mX=contourFinder.blobs[i].centroid.x *(1280.0f/640.0f);
        //mY=contourFinder.blobs[i].centroid.y *(800.0/480.0f);

        //find the lowest point of each Blob
        float maxim=0;  //maximum y
        int myMin=0;    //point that is lowest in y

        for (int p=0;p<contourFinder.blobs[i].nPts;p++){
            if (contourFinder.blobs[i].pts[p].y<maxim){
                    myMin=p;
                    maxim=contourFinder.blobs[i].pts[p].y;
            }
        }

        trackPoint.x=contourFinder.blobs[i].pts[myMin].x;
        trackPoint.y=contourFinder.blobs[i].pts[myMin].y;

        //if we have buffered values, delete the oldest one
        //now we must associate the currently tracked point with the closest point we already have
        //compare location, find closest already tracked point

        float shortestDist=trackDistance;

        int currentlyClosestBuffer=-1;  //set to -1, if it stays there, we probably have a new puppet to track!

        //the Blob search the closest point in the buffer
        //or the first empty buffer position? //TODO right?
        for (int l=0;l<NMAXBLOBS;l++){
            float dist=(trackPoint-trackPointBuffer[l][0]).length();//TODO why here a float declaration?
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
                l=NMAXBLOBS; //break out of for-loop
            }
        }

        //if we have found something...
        if (currentlyClosestBuffer>-1){
            //if we have buffered values, delete the oldest one
            //and insert the current Blob position into buffer
            if (trackPointBuffer[currentlyClosestBuffer].size()== TRACKBUFFER)
                trackPointBuffer[currentlyClosestBuffer].pop_back();

            trackPointBuffer[currentlyClosestBuffer].insert(trackPointBuffer[currentlyClosestBuffer].begin(),trackPoint);
        }
    }
}
*/

void testApp::applyMask(){

    //das eine minus das andere - wie geht?
    //ocvImage.absDiff(ocvDiff); // so gehts! - nee, so gehts nich
    //sondern:
    pixelBufferOne=ocvImage.getPixels();
    pixelBufferTwo=ocvDiff.getPixels();

    for (int i=0;i<640*480;i++){
        if (pixelBufferOne[i]-pixelBufferTwo[i]<0)
            pixelBufferThree[i]=pixelBufferOne[i];
        else{
            if (bInvertMask)
                pixelBufferThree[i]=0;
            else
                pixelBufferThree[i]=255;
        }
    }

    ocvImage.setFromPixels(pixelBufferThree,640,480);


}

/*void testApp::accumulateMask(){


pixelBufferOne=kinect.depthPixels;
pixelBufferTwo=ocvMask.getPixels();

for (int i=0;i<640*480;i++){
    if (pixelBufferOne[i]<pixelBufferTwo[i]){
        pixelBufferTwo[i]=pixelBufferOne[i];
    }
}


ocvMask.setFromPixels(pixelBufferTwo, 640,480);
//ocvDiff.setFromPixels(ocvMask.getPixels(),640,480);

}*/

/*void testApp::accumulateImage(){

    pixelBufferOne=kinect.depthPixels;
    pixelBufferTwo=ocvBufferedImage.getPixels();

    for (int i=0;i<640*480;i++){
        if (pixelBufferOne[i]<pixelBufferTwo[i]){
            pixelBufferTwo[i]=pixelBufferOne[i];
        }
    }

    ocvBufferedImage.setFromPixels(pixelBufferTwo, 640,480);
    //ocvImage.setFromPixels(kinect.depthPixels, 640,480);

}*/


//======================================DRAW FUNCTIONS

void testApp::draw(){

    ofClear(0,0,0,255); //screen black
    //ofNoFill();

    //glTexEnvf(GL_TEXTURE_FILTER_CONTROL_EXT, GL_TEXTURE_LOD_BIAS_EXT, slBut->sliderValue * 10.0);

    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    //ofSetLineWidth(1.0);

    //stageImage.draw(mainW,0);
    //kinect.draw(1920,0);

    //ofEnableAlphaBlending(); //ENABLE DISABLE ALPHABLENDING
    //ofDisableAlphaBlending();


    /*ofPushMatrix();

        ofSetHexColor(0xffff00);
        ofRect(100,800,320,200);

        ofSetHexColor(0xffffff);
        ofTranslate(100,100,0);
        ofScale(0.25,0.25,0.25);
        rgbaFbo.draw(0,0);

    ofPopMatrix();*/

    /*
    mX=  (mouseX - 100)*4;
    mY=  (mouseY - 400)*4;
    */

    //draw the grayscale kinect frame
    ofPushMatrix();
        ofTranslate(520,70);
        ofScale(0.75,0.75,0.75);
        ocvImage.draw(0,0);
        trackingBlobs.drawCountornFinder();
    ofPopMatrix();

            //was passiert hier eigentlich?

    //Draw Camera and Camtransform

    /*if (bAccumulateMask)
        ocvMask.draw(800,500);
    else
        ocvDiff.draw(800,500);*/

    //draw kinect webcam
    ofPushMatrix();
        ofTranslate(20,70);
        ofScale(0.75,0.75,0.75);
        kinect.draw(0,0);
    ofPopMatrix();

    //draw the mask after postProduction
    ofPushMatrix();
        ofTranslate(480+520+10,70); //TODO variables for positions
        ofScale(0.50,0.50,0.50);
        ocvDiff.draw(0,0);
    ofPopMatrix();

    //draw Mask
    ofPushMatrix();
        ofTranslate(480+520+10,240+70+10);//TODO here like :314
        ofScale(0.50,0.50,0.50);
        ocvMask.draw(0,0);
    ofPopMatrix();


    //calculate Buffered value
    //Vector3f trackPoint[NMAXBLOBS];

    //To make the tracking more fluid we take the average
    //of each tracked point from the trackPointBuffer
    /*for (int i=0;i<NMAXBLOBS;i++){
        for (int b=0;b<TRACKBUFFER;b++){
            //trackPoint[i]+=trackPointBuffer[i][b]/float(TRACKBUFFER);//warum sum(vect_i/num_tot)?
            trackPoint[i]+=trackPointBuffer[i][b];
            //trackPoint[i] = trackPointBuffer[i][b];
        }
        trackPoint[i]=trackPoint[i]/float(TRACKBUFFER);
    }*/

    //we draw the "id" number. It follow the tracked Blob
    //TODO: maybe too much iteration... or what made the tracking so slow?
    ofPushMatrix();
        ofTranslate(520,70);
        ofScale(0.75,0.75,0.75);

        trackingBlobs.averageHistory();
        trackingBlobs.drawIDs();

        /*for (int i=0;i<NMAXBLOBS;i++){
            //ofCircle(trackPoint[i].x,trackPoint[i].y,10);
            char buf[5];
            sprintf(buf,"%d",i);
            ofDrawBitmapString(buf,trackPoint[i].x,trackPoint[i].y,25);
        }*/

    ofPopMatrix();

    /*ofPushMatrix();
        ofTranslate(100,100);
        ofScale(0.5,0.5,0.5);

            //do das crazy Mathematics
            ofPushMatrix();

            ofPoint dstC[]={ofPoint(0,0),ofPoint(640,0),ofPoint(640,480),ofPoint(0,480)};
            ofPoint srcC[]={ofPoint(xC[0],yC[0]),ofPoint(xC[1],yC[1]),ofPoint(xC[2],yC[2]),ofPoint(xC[3],yC[3])};

            findHomography(srcC,dstC,cMat);
            glMultMatrixf(cMat);

            //ofSetColor(1.0,0.0,0.0,0.5);
            //kinect.drawDepth(0,0);

            for (int i=0;i<NMAXBLOBS;i++){
                ofPushMatrix();
                ofTranslate(trackPoint[i].x,trackPoint[i].y);
                ofSetHexColor(0x00ffff);
                ofCircle(0,0,10);
                glGetFloatv(GL_MODELVIEW_MATRIX,cMat);
                trackPoint[i]=cMat.getTranslation()/(cMat[15]*100.0);
                ofPopMatrix();
            }
            ofPopMatrix();

    ofPopMatrix();*/

/*
    for (int i=0;i<NMAXBLOBS;i++){

        ofPushMatrix();
            ofTranslate(100,100);

            //ofScale(2.0,2.0,2.0);
            ofSetHexColor(0xffff00);
            //dem magic numbers!
            trackPoint[i].x*=50;
            trackPoint[i].x+=1220;
            trackPoint[i].y*=50;
            trackPoint[i].y+=220;

            trackPoint[i]*=2.0;

            ofCircle(trackPoint[i].x,trackPoint[i].y,15);

        ofPopMatrix();

            rgbaFbo.begin();
                //drawFill( (mX/128)*128,(mY/128)*128);
                trackPoint[i].x*=4.0;
                trackPoint[i].y*=3.0;*/
                //drawFill( trackPoint[i].x, trackPoint[i].y );
                //draw connections
                /*if (connectors[i]->color==Vector4f(0.5,0,0,1)){
                    for (int j=0;j<NMAXBLOBS;j++){
                        if (connected[j]->color==Vector4f(0,0.5,0,1)){
                            drawConnect(trackPoint[i].x, trackPoint[i].y,trackPoint[j].x, trackPoint[j].y);
                        }
                    }
                }*/


                //if (i>0)
                //    ofLine(trackPoint[i].x, trackPoint[i].y,trackPoint[i-1].x, trackPoint[i-1].y  );

                //drawFill( (int(trackPoint.x)/128)*128, (int(trackPoint.y)/128)*128 );

                /*if (bMockup){
                        ofRotate(45);
                    mockup.draw(0,500);
                }*/

            //rgbaFbo.end();
    //}


    //ofNoFill();

    //draw the "textual interface"
    sprintf(valueStr, "erode a/q  %i", erodeAmount);
    valueNumber.drawString(valueStr, 20,560);
    sprintf(valueStr, "dilate s/w  %i", dilateAmount);
    valueNumber.drawString(valueStr, 20,600);
    sprintf(valueStr, "threshhold d/e  %i", threshold);
    valueNumber.drawString(valueStr, 20,640);
    sprintf(valueStr, "minArea Blob f/r  %i", minDimBlob);
    valueNumber.drawString(valueStr, 20,680);

}


//=============================================BUTTONS FUNCTIONS

/*void testApp::checkConnections(Actor* other){

    //color toggle

    for (int i=0;i<NMAXBLOBS;i++){
        if (other==connected[i]){
            if (connected[i]->color==Vector4f(0,0.5,0,1)){
                connected[i]->color=Vector4f(0,0,0.5,1);
            }else{
                connected[i]->color=Vector4f(0,0.5,0,1);
            }
        }
    }

    //color toggle

    for (int i=0;i<NMAXBLOBS;i++){
        if (other==connectors[i]){
            if (connectors[i]->color==Vector4f(0.5,0,0,1)){
                connectors[i]->color=Vector4f(0.5,0.5,0,1);
            }else{
                connectors[i]->color=Vector4f(0.5,0,0,1);
            }
        }
    }


}*/

//=============================================INPUT FUNCTIONS
void testApp::keyPressed(int key){

}

void testApp::keyReleased(int key){


    if (key=='m'){
        //start mask accumulation
        //bAccumulateMask=!bAccumulateMask; //TODO is this right?
        //if we just started, reset the initial Mask
        //if (bAccumulateMask){
            ocvMask.setFromPixels(kinect.depthPixels, 640,480);
        //otherwise, do the calculations... like erode and dilate
        //}else{
            postProcessMask();
        //}
    }

    //Reset trackPointBuffers
    if (key=='z'){
        /*cout << "resetting trackPoint Buffer" << endl;
        //for each Puppet potentially to be tracked
        for (int i=0;i<NMAXBLOBS;i++){
            trackPointBuffer[i].clear();
            //set up a buffer as big as TRACKBUFFER
            for (int b=0;b<TRACKBUFFER;b++){
                trackPointBuffer[i].push_back(Vector3f(0,0,0));
            }
        }*/
        trackingBlobs.resetBuffer();

    }

    if (key=='w'){
        dilateAmount++;
        postProcessMask();
    }
    if (key=='q'){
        erodeAmount++;
        postProcessMask();
    }
    if (key=='s'){
        dilateAmount--;
        postProcessMask();
    }
    if (key=='a'){
        erodeAmount--;
        postProcessMask();
    }
    if (key=='e'){
        threshold++;
    }
        if (key=='d'){
        threshold--;
    }
    if (key=='f'){
        minDimBlob-=100;
    }
        if (key=='r'){
        minDimBlob+=100;
    }

}

void testApp::mouseMoved(int x, int y ){

    mX=x;
    mY=y;
}

void testApp::mouseDragged(int x, int y, int button){

}

void testApp::mousePressed(int x, int y, int button){

}

void testApp::mouseReleased(int x, int y, int button){

}

void testApp::windowResized(int w, int h){

}


//============================================SPECIAL FUNCTIONS


//-----------------------------------------
void testApp::postProcessMask(){

    ocvDiff.setFromPixels(ocvMask.getPixels(),640,480);

    for (int i=0;i<dilateAmount; i++)
        ocvDiff.dilate();

    for (int i=0;i<erodeAmount; i++)
        ocvDiff.erode();

    //ocvDiff.blurGaussian(blurAmount);


}

