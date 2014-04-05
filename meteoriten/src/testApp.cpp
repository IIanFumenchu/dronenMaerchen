#include "testApp.h"

//--------------------------------------------------------------

testApp::testApp(){}

testApp::~testApp(){

    TiXmlDocument doc;
    TiXmlDeclaration * decl = new TiXmlDeclaration( "1.0", "", "" );
    doc.LinkEndChild( decl );
    TiXmlElement * root = new TiXmlElement( "MsbKinect" );
    doc.LinkEndChild( root );

    TiXmlElement* myElement=save(root);
    root->LinkEndChild(myElement);
    doc.SaveFile("userConfig.xml");

}


void testApp::setup(){

    //ofSetFrameRate(25);


    int offsetX=300;
    int offsetY=300;


    xP[0]=0+offsetX;
    xP[1]=1280.0-offsetX;
    xP[2]=1280-offsetX;
    xP[3]=0+offsetX;
    yP[0]=0+offsetY;
    yP[1]=0+offsetY;
    yP[2]=800-offsetY;
    yP[3]=800-offsetY;

    xP2[0]=0+offsetX;
    xP2[1]=1280-offsetX;
    xP2[2]=1280-offsetX;
    xP2[3]=0+offsetX;
    yP2[0]=0+offsetY;
    yP2[1]=0+offsetY;
    yP2[2]=800-offsetY;
    yP2[3]=800-offsetY;

    xC[0]=0;
    xC[1]=640;
    xC[2]=640;
    xC[3]=0;
    yC[0]=0;
    yC[1]=0;
    yC[2]=480;
    yC[3]=480;

    currentFrame=0;

    mX=0;
    mY=0;

    mainW = 1920;
    mainH = 1080;
    firstW = 1280;
    firstH = 800;
    secondW = 1280;
    secondH = 800;

    threshold=40;
    trackDistance=100;
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

    //set up tracking buffers
    //for each Puppet potentially to be tracked
    for (int i=0;i<MAXPUPPE;i++){
        //set up a buffer as big as BUFFER
        for (int b=0;b<TRACKBUFFER;b++){
            trackPointBuffer[i].push_back(Vector3f(0,0,0));
        }
    }


    //ofSetFrameRate(25);


    ofDisableArbTex();

    rgbaFbo.allocate(1280, 800, GL_RGBA); // with alpha, 8 bits red, 8 bits green, 8 bits blue, 8 bits alpha, from 0 to 255 in 256 steps

    rgbaFbo.begin();

	ofClear(255,255,255, 0);

    rgbaFbo.end();


    //stage simulation
    stageImage.loadImage("stage.tga");
    mockup.loadMovie("mockup.mp4");

    //stageImage.getTextureReference().texData.textureTarget=GL_TEXTURE_2D;

    msbSetup();

    loadSettings();

    interfaceSetup();


}

void testApp::msbSetup(){

    renderer=Renderer::getInstance();
    input=Input::getInstance();

    renderer->loadPreferences();

    renderer->windowX=SCREENRESX;
    renderer->windowY=SCREENRESY;

    renderer->screenX=SCREENRESX;
    renderer->screenY=SCREENRESY;

    renderer->setup();

    renderer->camActor=new Actor;
    renderer->camActor->setLocation(Vector3f(0,0,-5));
    renderer->camActor->postLoad();

    registerProperties();
}

void testApp::registerProperties(){

    createMemberID("XP",&xP,this);
    createMemberID("YP",&yP,this);

    createMemberID("XP2",&xP2,this);
    createMemberID("YP2",&yP2,this);

    createMemberID("XC",&xC,this);
    createMemberID("YC",&yC,this);

    createMemberID("TRACKDISTANCE",&trackDistance,this);
    createMemberID("THRESHOLD",&threshold,this);
    createMemberID("LINEWIDTH",&lineWidth,this);

    createMemberID("DILATEAMOUNT",&dilateAmount,this);
    createMemberID("ERODEAMOUNT",&erodeAmount,this);
    createMemberID("BLURAMOUNT",&blurAmount,this);
    createMemberID("IMAGEBUFFER",&imageBuffer,this);


}

//--------------------------------------------------------------
void testApp::update(){

    rectSize=lineWidth*64;

    currentFrame++;
    kinect.update();
    renderer->update();
    mockup.update();

    rgbaFbo.begin();
        ofClear(0,0,0,0);
		if (bDrawGrid)
            drawGrid();
    rgbaFbo.end();

    if (bAccumulateMask)
        accumulateMask();

    //buffer image
    if(currentFrame%imageBuffer==0){
        //reset image to be accumulated
        ocvImage.setFromPixels(ocvBufferedImage.getPixels(),640,480);
        ocvBufferedImage.setFromPixels(kinect.depthPixels,640,480);
    }else{
        //build image
        accumulateImage();
    }


    //do all the stuff all the time
    applyMask();
    trackPoints();

}

void testApp::trackPoints(){

    Vector3f trackPoint;

        //debug - use mouse coordinates!
    /*

        trackPoint.x=mX-1000;
        trackPoint.y=mY;

        if (trackPointBuffer[0].size()== TRACKBUFFER)
            trackPointBuffer[0].pop_back();

        trackPointBuffer[0].insert(trackPointBuffer[0].begin(),trackPoint);
    */


    //fill Track Point Buffer


    //ocvImage.threshold(threshold,true);
    contourFinder.findContours(ocvImage,200,200000,MAXPUPPE,false,true);
    for(int i = 0; i < contourFinder.nBlobs; i++) {
        mX=contourFinder.blobs[i].centroid.x *(1280.0f/640.0f);
        mY=contourFinder.blobs[i].centroid.y *(800.0/480.0f);

        //find point that is highest in y
        float maxim=0;  //maximum y
        int myMin=0;    //point that is highest in y

        for (int p=0;p<contourFinder.blobs[i].nPts;p++){
            if (contourFinder.blobs[i].pts[p].y>maxim){
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

        for (int l=0;l<MAXPUPPE;l++){
            float dist=(trackPoint-trackPointBuffer[l][0]).length();
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
                l=MAXPUPPE; //break out of for-loop
            }
        }

        //if we haven't found anything...
        if (currentlyClosestBuffer>-1){
            //if we have buffered values, delete the oldest one
            if (trackPointBuffer[currentlyClosestBuffer].size()== TRACKBUFFER)
                trackPointBuffer[currentlyClosestBuffer].pop_back();
            //insert new value into buffer
            trackPointBuffer[currentlyClosestBuffer].insert(trackPointBuffer[currentlyClosestBuffer].begin(),trackPoint);
        }
    }
}


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

void testApp::accumulateMask(){


pixelBufferOne=kinect.depthPixels;
pixelBufferTwo=ocvMask.getPixels();

for (int i=0;i<640*480;i++){
    if (pixelBufferOne[i]<pixelBufferTwo[i]){
        pixelBufferTwo[i]=pixelBufferOne[i];
    }
}


ocvMask.setFromPixels(pixelBufferTwo, 640,480);
//ocvDiff.setFromPixels(ocvMask.getPixels(),640,480);

}


void testApp::accumulateImage(){

    pixelBufferOne=kinect.depthPixels;
    pixelBufferTwo=ocvBufferedImage.getPixels();

    for (int i=0;i<640*480;i++){
        if (pixelBufferOne[i]<pixelBufferTwo[i]){
            pixelBufferTwo[i]=pixelBufferOne[i];
        }
    }

    ocvBufferedImage.setFromPixels(pixelBufferTwo, 640,480);
    //ocvImage.setFromPixels(kinect.depthPixels, 640,480);

}

//--------------------------------------------------------------
void testApp::drawGrid(){

    ofSetHexColor(0xffffff);
    ofNoFill();
    ofSetCircleResolution(128);
    ofSetLineWidth(lineWidth);
    //ofCircle(mX,mY,200);

    for(int grY=0; grY<=800; grY+=128){
            for(int grX=0; grX<=1280; grX+=128){
                ofRect(grX,grY,128,128);
            }
        }

}

void testApp::drawFill(int x, int y){

    ofSetHexColor(0xffffff);
    ofFill();
    ofSetLineWidth(lineWidth);
    //ofRect(x,y,rectSize,rectSize);
    ofRect(x-threshold,y-threshold,threshold*2.0,threshold*2.0);

}

void testApp::drawConnect(int x1, int y1, int x2, int y2){

    ofSetHexColor(0xffffff);
    ofFill();
    ofSetLineWidth(lineWidth);
    ofLine(x1,y1,x2,y2);

}


void testApp::draw(){

    ofClear(0,0,0,255);
    ofNoFill();

    //glTexEnvf(GL_TEXTURE_FILTER_CONTROL_EXT, GL_TEXTURE_LOD_BIAS_EXT, slBut->sliderValue * 10.0);

    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    ofSetLineWidth(1.0);

    //stageImage.draw(mainW,0);
    //kinect.draw(1920,0);

    ofEnableAlphaBlending();


    ofPushMatrix();

        ofSetHexColor(0xffff00);
        ofRect(100,800,320,200);

        ofSetHexColor(0xffffff);
        ofTranslate(100,800,0);
        ofScale(0.25,0.25,0.25);
        rgbaFbo.draw(0,0);

    ofPopMatrix();

    /*
    mX=  (mouseX - 100)*4;
    mY=  (mouseY - 400)*4;
    */

    ofPushMatrix();
    ofTranslate(600,100);
    ofScale(0.5,0.5,1.0);
    ocvImage.draw(0,0);
    contourFinder.draw();

    ofPopMatrix();

            //was passiert hier eigentlich?

    //Draw Camera and Camtransform

    if (bAccumulateMask)
        ocvMask.draw(1000,500);
    else
        ocvDiff.draw(1000,500);

    kinect.draw(1000,0);


    //calculate Buffered value
    Vector3f trackPoint[MAXPUPPE];

    //for all trackPoints
    for (int i=0;i<MAXPUPPE;i++){
        //add up all Buffered values
        for (int b=0;b<TRACKBUFFER;b++){
            trackPoint[i]+=trackPointBuffer[i][b]/float(TRACKBUFFER);
        }
    }

    for (int i=0;i<MAXPUPPE;i++){
        ofCircle(trackPoint[i].x+1000,trackPoint[i].y,10);
        char buf[5];
        sprintf(buf,"%d",i);
        ofDrawBitmapString(buf,trackPoint[i].x+1000,trackPoint[i].y,10);
    }

    ofPushMatrix();
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

            for (int i=0;i<MAXPUPPE;i++){
                ofPushMatrix();
                ofTranslate(trackPoint[i].x,trackPoint[i].y);
                ofSetHexColor(0x00ffff);
                ofCircle(0,0,10);
                glGetFloatv(GL_MODELVIEW_MATRIX,cMat);
                trackPoint[i]=cMat.getTranslation()/(cMat[15]*100.0);
                ofPopMatrix();
            }
            ofPopMatrix();

    ofPopMatrix();


    for (int i=0;i<MAXPUPPE;i++){

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
                trackPoint[i].y*=3.0;
                drawFill( trackPoint[i].x, trackPoint[i].y );
                //draw connections
                if (connectors[i]->color==Vector4f(0.5,0,0,1)){
                    for (int j=0;j<MAXPUPPE;j++){
                        if (connected[j]->color==Vector4f(0,0.5,0,1)){
                            drawConnect(trackPoint[i].x, trackPoint[i].y,trackPoint[j].x, trackPoint[j].y);
                        }
                    }
                }


                //if (i>0)
                //    ofLine(trackPoint[i].x, trackPoint[i].y,trackPoint[i-1].x, trackPoint[i-1].y  );

                //drawFill( (int(trackPoint.x)/128)*128, (int(trackPoint.y)/128)*128 );

                if (bMockup){
                        ofRotate(45);
                    mockup.draw(0,500);
                }

            rgbaFbo.end();
    }


    ofNoFill();


    //DRAW ALL TEH GRIDZ!!!11!


    //Draw first Projection Grid
    ofPushMatrix();

        //Draw tiny red rectangle in which we draw first Projection Grid
        ofSetHexColor(0xff0000);
        ofRect(100,100,320,200);

        ofTranslate(100,100,0);
        ofScale(0.25,0.25,0.25);


        ofPushMatrix();

            //do das crazy Mathematics
            ofPoint src[]={ofPoint(0,0),ofPoint(1280,0),ofPoint(1280,800),ofPoint(0,800)};
            ofPoint dst[]={ofPoint(xP[0],yP[0]),ofPoint(xP[1],yP[1]),ofPoint(xP[2],yP[2]),ofPoint(xP[3],yP[3])};

            findHomography(src,dst,pMat1);
            glMultMatrixf(pMat1);
            ofSetHexColor(0xffffff);

            rgbaFbo.draw(0,0);

        ofPopMatrix();
    ofPopMatrix();

    //Draw second Projection Grid
    ofPushMatrix();

        //Draw tiny red rectangle in which we draw second Projection Grid
        ofSetHexColor(0xff0000);
        ofRect(100,500,320,200);

        ofTranslate(100,500,0);
        ofScale(0.25,0.25,0.25);

        //do das crazy Mathematics
        ofPushMatrix();

            ofPoint srcP2[]={ofPoint(0,0),ofPoint(1280,0),ofPoint(1280,800),ofPoint(0,800)};
            ofPoint dstP2[]={ofPoint(xP2[0],yP2[0]),ofPoint(xP2[1],yP2[1]),ofPoint(xP2[2],yP2[2]),ofPoint(xP2[3],yP2[3])};


            findHomography(srcP2,dstP2,pMat2);
            glMultMatrixf(pMat2);
            ofSetHexColor(0xffffff);

            rgbaFbo.draw(0,0);

        ofPopMatrix();

    ofPopMatrix();

    ofPushMatrix();

        //draw final translated first Grid
        ofTranslate(mainW,0);//main monitor resolution
        ofPushMatrix();
            glMultMatrixf(pMat1);
            ofSetColor(255,255,255);
            rgbaFbo.draw(0,0);
        ofPopMatrix();

        //draw final translated second Grid
        ofTranslate(firstW,0); //first additional screen resolution
        ofPushMatrix();
            glMultMatrixf(pMat2);
            ofSetColor(255,255,255);
            rgbaFbo.draw(0,0);
        ofPopMatrix();


    ofPopMatrix();

    renderer->backgroundColor.a=0;

    renderer->draw();

}


void testApp::postProcessMask(){

    ocvDiff.setFromPixels(ocvMask.getPixels(),640,480);

    for (int i=0;i<dilateAmount; i++)
        ocvDiff.dilate();

    for (int i=0;i<erodeAmount; i++)
        ocvDiff.erode();

    //ocvDiff.blurGaussian(blurAmount);


}

void testApp::trigger(Actor* other){

    if (other->name=="thresholdSlider"){

        //cout << ((SliderButton*)other)->sliderValue << endl;
        threshold=((SliderButton*)other)->sliderValue * 255.0;

    }

    if (other->name=="stefano"){

        cout << "hippie!!!" << endl;

    }

    if (other->name=="threshold"){
        slBut->sliderValue=threshold/255.0;
    }

    if (other->name=="erode" ||other->name=="dilate" ||other->name=="blur"){

        postProcessMask();
    }

    //erstes Projektionsgrid

    if (other->name=="ecke1"){
        xP[0]=other->location.x-mainW+16;
        yP[0]=other->location.y+16;
    }

    if (other->name=="ecke2"){
        xP[1]=other->location.x-mainW+16;
        yP[1]=other->location.y+16;
    }

    if (other->name=="ecke3"){
        xP[2]=other->location.x-mainW+16;
        yP[2]=other->location.y+16;
    }

    if (other->name=="ecke4"){
        xP[3]=other->location.x-mainW+16;
        yP[3]=other->location.y+16;
    }


    //zweites Projektionsgrid

    if (other->name=="eckeB1"){
        xP2[0]=other->location.x-(mainW+firstW)+16;
        yP2[0]=other->location.y+16;
        cout << "yeah!" << endl;
    }

    if (other->name=="eckeB2"){
        xP2[1]=other->location.x-(mainW+firstW)+16;
        yP2[1]=other->location.y+16;
    }

    if (other->name=="eckeB3"){
        xP2[2]=other->location.x-(mainW+firstW)+16;
        yP2[2]=other->location.y+16;
    }

    if (other->name=="eckeB4"){
        xP2[3]=other->location.x-(mainW+firstW)+16;
        yP2[3]=other->location.y+16;
    }







    //Kameragrid

    if (other->name=="ecke1c"){
        xC[0]=other->location.x-1000+16;
        yC[0]=other->location.y+16;
    }

    if (other->name=="ecke2c"){
        xC[1]=other->location.x-1000+16;
        yC[1]=other->location.y+16;
    }

    if (other->name=="ecke3c"){
        xC[2]=other->location.x-1000+16;
        yC[2]=other->location.y+16;
    }

    if (other->name=="ecke4c"){
        xC[3]=other->location.x-1000+16;
        yC[3]=other->location.y+16;
    }

    checkConnections(other);

}

void testApp::checkConnections(Actor* other){

    //color toggle

    for (int i=0;i<MAXPUPPE;i++){
        if (other==connected[i]){
            if (connected[i]->color==Vector4f(0,0.5,0,1)){
                connected[i]->color=Vector4f(0,0,0.5,1);
            }else{
                connected[i]->color=Vector4f(0,0.5,0,1);
            }
        }
    }

    //color toggle

    for (int i=0;i<MAXPUPPE;i++){
        if (other==connectors[i]){
            if (connectors[i]->color==Vector4f(0.5,0,0,1)){
                connectors[i]->color=Vector4f(0.5,0.5,0,1);
            }else{
                connectors[i]->color=Vector4f(0.5,0,0,1);
            }
        }
    }


}

//--------------------------------------------------------------
void testApp::keyPressed(int key){


        if (key=='w'){
            xP2[selectedPoint-1]+=0.0;
            yP2[selectedPoint-1]-=0.5;
        }

        if (key=='a'){
            xP2[selectedPoint-1]-=0.5;
            yP2[selectedPoint-1]-=0.0;
        }

        if (key=='s'){
            xP2[selectedPoint-1]-=0.0;
            yP2[selectedPoint-1]+=0.5;
        }

        if (key=='d'){
            xP2[selectedPoint-1]+=0.5;
            yP2[selectedPoint-1]-=0.0;
        }


    input->normalKeyDown(key,mouseX,mouseY);
}
//--------------------------------------------------------------
void testApp::keyReleased(int key){

    input->keyUp(key,mouseX,mouseY);

    if (input->bTextInput)
        return;

    if (key=='1'){
        selectedPoint=1;
    }

    if (key=='2'){
        selectedPoint=2;
    }

    if (key=='3'){
        selectedPoint=3;
    }

    if (key=='4'){
        selectedPoint=4;
    }

    if (key=='m'){
        //start mask accumulation
        bAccumulateMask=!bAccumulateMask;
        //if we just started, reset the initial Mask
        if (bAccumulateMask){
            ocvMask.setFromPixels(kinect.depthPixels, 640,480);
        //otherwise, do the calculations like erode and stuff
        }else{
            postProcessMask();
        }
    }

    if (key=='o'){   //mockup
        bMockup=!bMockup;
        if (bMockup)
            mockup.play();
    }

    if (key=='g'){
        bDrawGrid=!bDrawGrid;
    }

    //Reset trackPointBuffers
    if (key=='r'){
        cout << "resetting trackPoint Buffer" << endl;
        //for each Puppet potentially to be tracked
        for (int i=0;i<MAXPUPPE;i++){
            trackPointBuffer[i].clear();
            //set up a buffer as big as TRACKBUFFER
            for (int b=0;b<TRACKBUFFER;b++){
                trackPointBuffer[i].push_back(Vector3f(0,0,0));
            }
        }

    }

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

    mX=x;
    mY=y;

    input->moveMouse(x,y);
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

    input->dragMouse(x,y);
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

    input->pressedMouse(button,0,x,y);

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

    input->pressedMouse(button,1,x,y);

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}


void testApp::findHomography(ofPoint src[4], ofPoint dst[4], float homography[16]){
	// arturo castro - 08/01/2010
	//
	// create the equation system to be solved
	//
	// from: Multiple View Geometry in Computer Vision 2ed
	//       Hartley R. and Zisserman A.
	//
	// x' = xH
	// where H is the homography: a 3 by 3 matrix
	// that transformed to inhomogeneous coordinates for each point
	// gives the following equations for each point:
	//
	// x' * (h31*x + h32*y + h33) = h11*x + h12*y + h13
	// y' * (h31*x + h32*y + h33) = h21*x + h22*y + h23
	//
	// as the homography is scale independent we can let h33 be 1 (indeed any of the terms)
	// so for 4 points we have 8 equations for 8 terms to solve: h11 - h32
	// after ordering the terms it gives the following matrix
	// that can be solved with gaussian elimination:
	float P[8][9]={
		{-src[0].x, -src[0].y, -1,   0,   0,  0, src[0].x*dst[0].x, src[0].y*dst[0].x, -dst[0].x }, // h11
		{  0,   0,  0, -src[0].x, -src[0].y, -1, src[0].x*dst[0].y, src[0].y*dst[0].y, -dst[0].y }, // h12
		{-src[1].x, -src[1].y, -1,   0,   0,  0, src[1].x*dst[1].x, src[1].y*dst[1].x, -dst[1].x }, // h13
		{  0,   0,  0, -src[1].x, -src[1].y, -1, src[1].x*dst[1].y, src[1].y*dst[1].y, -dst[1].y }, // h21
		{-src[2].x, -src[2].y, -1,   0,   0,  0, src[2].x*dst[2].x, src[2].y*dst[2].x, -dst[2].x }, // h22
		{  0,   0,  0, -src[2].x, -src[2].y, -1, src[2].x*dst[2].y, src[2].y*dst[2].y, -dst[2].y }, // h23
		{-src[3].x, -src[3].y, -1,   0,   0,  0, src[3].x*dst[3].x, src[3].y*dst[3].x, -dst[3].x }, // h31
		{  0,   0,  0, -src[3].x, -src[3].y, -1, src[3].x*dst[3].y, src[3].y*dst[3].y, -dst[3].y }, // h32
	};
	gaussian_elimination(&P[0][0],9);
	// gaussian elimination gives the results of the equation system
	// in the last column of the original matrix.
	// opengl needs the transposed 4x4 matrix:
	float aux_H[]={ P[0][8],P[3][8],0,P[6][8], // h11  h21 0 h31
		P[1][8],P[4][8],0,P[7][8], // h12  h22 0 h32
		0      ,      0,0,0,       // 0    0   0 0
		P[2][8],P[5][8],0,1};      // h13  h23 0 h33
	for(int i=0;i<16;i++) homography[i] = aux_H[i];
}

void testApp::gaussian_elimination(float *input, int n){
	// arturo castro - 08/01/2010
	//
	// ported to c from pseudocode in
	// [http://en.wikipedia.org/wiki/Gaussian-elimination](http://en.wikipedia.org/wiki/Gaussian-elimination)
	float * A = input;
	int i = 0;
	int j = 0;
	int m = n-1;
	while (i < m && j < n){
		// Find pivot in column j, starting in row i:
		int maxi = i;
		for(int k = i+1; k<m; k++){
			if(fabs(A[k*n+j]) > fabs(A[maxi*n+j])){
				maxi = k;
			}
		}
		if (A[maxi*n+j] != 0){
			//swap rows i and maxi, but do not change the value of i
			if(i!=maxi)
				for(int k=0;k<n;k++){
					float aux = A[i*n+k];
					A[i*n+k]=A[maxi*n+k];
					A[maxi*n+k]=aux;
				}
			//Now A[i,j] will contain the old value of A[maxi,j].
			//divide each entry in row i by A[i,j]
			float A_ij=A[i*n+j];
			for(int k=0;k<n;k++){
				A[i*n+k]/=A_ij;
			}
			//Now A[i,j] will have the value 1.
			for(int u = i+1; u< m; u++){
				//subtract A[u,j] * row i from row u
				float A_uj = A[u*n+j];
				for(int k=0;k<n;k++){
					A[u*n+k]-=A_uj*A[i*n+k];
				}
				//Now A[u,j] will be 0, since A[u,j] - A[i,j] * A[u,j] = A[u,j] - 1 * A[u,j] = 0.
			}
			i++;
		}
		j++;
	}
	//back substitution
	for(int i=m-2;i>=0;i--){
		for(int j=i+1;j<n-1;j++){
			A[i*n+m]-=A[i*n+j]*A[j*n+m];
			//A[i*n+j]=0;
		}
	}
}


void testApp::loadSettings(){

    //load settings
    TiXmlDocument doc( "userConfig.xml" );
    if (!doc.LoadFile()) return;
    TiXmlHandle hDoc(&doc);
    TiXmlElement * element;
    TiXmlHandle hRoot(0);
    element=hDoc.FirstChildElement().Element();
    if (!element) return;

    hRoot=TiXmlHandle(element);
    element=hRoot.FirstChild( "Actor" ).Element();
    int listPos=0;
    string myType;

      for( ; element!=NULL; element=element->NextSiblingElement("Actor"))
        {
        myType=element->GetText();
        cout << "Loading property type: " << myType << endl;
        load(element);
        listPos++;
        }
}



void testApp::interfaceSetup(){
    //Adding MSB content
    BasicButton *but;

    cornerSetup();

    //anderes


    but= new TextInputButton();
    but->name="threshold";
    but->buttonProperty="THRESHOLD";
    but->bDrawName=true;
    but->tooltip="";
    but->setLocation(Vector3f(500,700,0));
    but->scale.x=130;
    but->scale.y=20;
    but->textureID="icon_flat";
    but->parent=this;
    but->color=Vector4f(0.5,0.5,0.5,1.0);
    but->setup();
    renderer->buttonList.push_back(but);

    but= new TextInputButton();
    but->name="lineWidth";
    but->buttonProperty="LINEWIDTH";
    but->bDrawName=true;
    but->tooltip="";
    but->setLocation(Vector3f(700,700,0));
    but->scale.x=130;
    but->scale.y=20;
    but->textureID="icon_flat";
    but->parent=this;
    but->color=Vector4f(0.5,0.5,0.5,1.0);
    but->setup();
    renderer->buttonList.push_back(but);

    but= new TextInputButton();
    but->name="transformX";
    but->buttonProperty="XP";
    but->bDrawName=true;
    but->tooltip="";
    but->setLocation(Vector3f(500,730,0));
    but->scale.x=130;
    but->scale.y=12;
    but->textureID="icon_flat";
    but->parent=this;
    but->color=Vector4f(0.5,0.5,0.5,1.0);
    but->setup();
    renderer->buttonList.push_back(but);

    but= new TextInputButton();
    but->name="transformY";
    but->buttonProperty="YP";
    but->bDrawName=true;
    but->tooltip="";
    but->setLocation(Vector3f(500,750,0));
    but->scale.x=130;
    but->scale.y=12;
    but->textureID="icon_flat";
    but->parent=this;
    but->color=Vector4f(0.5,0.5,0.5,1.0);
    but->setup();
    renderer->buttonList.push_back(but);

    but= new TextInputButton();
    but->name="dilate";
    but->buttonProperty="DILATEAMOUNT";
    but->bDrawName=true;
    but->tooltip="";
    but->setLocation(Vector3f(500,790,0));
    but->scale.x=130;
    but->scale.y=12;
    but->textureID="icon_flat";
    but->parent=this;
    but->color=Vector4f(0.5,0.5,0.5,1.0);
    but->setup();
    renderer->buttonList.push_back(but);

    but= new TextInputButton();
    but->name="erode";
    but->buttonProperty="ERODEAMOUNT";
    but->bDrawName=true;
    but->tooltip="";
    but->setLocation(Vector3f(500,810,0));
    but->scale.x=130;
    but->scale.y=12;
    but->textureID="icon_flat";
    but->parent=this;
    but->color=Vector4f(0.5,0.5,0.5,1.0);
    but->setup();
    renderer->buttonList.push_back(but);

    but= new TextInputButton();
    but->name="blur";
    but->buttonProperty="BLURAMOUNT";
    but->bDrawName=true;
    but->tooltip="";
    but->setLocation(Vector3f(500,830,0));
    but->scale.x=130;
    but->scale.y=12;
    but->textureID="icon_flat";
    but->parent=this;
    but->color=Vector4f(0.5,0.5,0.5,1.0);
    but->setup();
    renderer->buttonList.push_back(but);


    but= new TextInputButton();
    but->name="trackDistance";
    but->buttonProperty="TRACKDISTANCE";
    but->bDrawName=true;
    but->tooltip="";
    but->setLocation(Vector3f(500,860,0));
    but->scale.x=130;
    but->scale.y=12;
    but->textureID="icon_flat";
    but->parent=this;
    but->color=Vector4f(0.5,0.5,0.5,1.0);
    but->setup();
    renderer->buttonList.push_back(but);

    but= new TextInputButton();
    but->name="imageBuffer";
    but->buttonProperty="IMAGEBUFFER";
    but->bDrawName=true;
    but->tooltip="";
    but->setLocation(Vector3f(500,890,0));
    but->scale.x=130;
    but->scale.y=12;
    but->textureID="icon_flat";
    but->parent=this;
    but->color=Vector4f(0.5,0.5,0.5,1.0);
    but->setup();
    renderer->buttonList.push_back(but);

    slBut= new SliderButton();
    slBut->name="thresholdSlider";
    slBut->bDrawName=false;
    slBut->tooltip="set kinect distance";
    slBut->setLocation(Vector3f(600,620,0));
    slBut->scale.x=320;
    slBut->scale.y=16;
    slBut->bVertical=false;
    slBut->textureID="icon_flat";
    slBut->color=Vector4f(0.5,0.5,0.5,1.0);
    slBut->parent=this;
    slBut->setup();
    slBut->sliderValue=threshold/255.0;
    renderer->buttonList.push_back(slBut);


    connectSetup();

}

void testApp::connectSetup(){

    BasicButton* but;

    for (int i=0;i<MAXPUPPE;i++){
        but= new AssignButton();
        but->name="connect";
        but->bDrawName=false;
        but->tooltip="";
        but->setLocation(Vector3f(500+i*(but->scale.x+10),920,0));
        but->scale.x=32;
        but->scale.y=32;
        but->parent=this;
        but->color=Vector4f(0.5,0.5,0.0,1.0);
        but->setup();
        renderer->buttonList.push_back(but);
        connectors.push_back(but);
    }

    for (int i=0;i<MAXPUPPE;i++){
        but= new AssignButton();
        but->name="connect";
        but->bDrawName=false;
        but->tooltip="";
        but->setLocation(Vector3f(500+i*(but->scale.x+10),980,0));
        but->scale.x=32;
        but->scale.y=32;
        but->textureID="icon_flat";
        but->parent=this;
        but->color=Vector4f(0.0,0.0,0.5,1);
        but->setup();
        renderer->buttonList.push_back(but);
        connected.push_back(but);
    }
}


void testApp::cornerSetup(){

    BasicButton *but;

    //eckKnoepfe
    Vector3f eckScale=Vector3f(32,32,1);
    Vector4f eckColor=Vector4f(0,1,1,1.0);


        //corners
    but= new AssignButton;
    but->name="ecke1";
    but->bDrawName=true;
    but->bDragable=true;
    but->tooltip="drag Me!";
    but->setLocation(Vector3f(xP[0]+mainW-16,yP[0]-16,0));
    but->scale=eckScale;
    but->color=eckColor;
    but->bTriggerWhileDragging=true;
    but->bResetAfterDrag=false;
    but->textureID="ecke";
    but->parent=this;
    but->setup();
    renderer->buttonList.push_back(but);

    but= new AssignButton;
    but->name="ecke2";
    but->bDrawName=true;
    but->bDragable=true;
    but->tooltip="drag Me!";
    but->setLocation(Vector3f(xP[1]+mainW-16,yP[1]-16,0));
    but->scale=eckScale;
    but->color=eckColor;
    but->bTriggerWhileDragging=true;
    but->bResetAfterDrag=false;
    but->textureID="ecke";
    but->parent=this;
    but->setup();
    renderer->buttonList.push_back(but);

    but= new AssignButton;
    but->name="ecke3";
    but->bDrawName=true;
    but->bDragable=true;
    but->tooltip="drag Me!";
    but->setLocation(Vector3f(xP[2]+mainW-16,yP[2]-16,0));
    but->scale=eckScale;
    but->color=eckColor;
    but->bTriggerWhileDragging=true;
    but->bResetAfterDrag=false;
    but->textureID="ecke";
    but->parent=this;
    but->setup();
    renderer->buttonList.push_back(but);

    but= new AssignButton;
    but->name="ecke4";
    but->bDrawName=true;
    but->bDragable=true;
    but->tooltip="drag Me!";
    but->setLocation(Vector3f(xP[3]+mainW-16,yP[3]-16,0));
    but->scale=eckScale;
    but->color=eckColor;
    but->bTriggerWhileDragging=true;
    but->bResetAfterDrag=false;
    but->textureID="ecke";
    but->parent=this;
    but->setup();
    renderer->buttonList.push_back(but);

    eckColor=Vector4f(1,1,0,1.0);




    //projector2
    but= new AssignButton;
    but->name="eckeB1";
    but->bDrawName=true;
    but->bDragable=true;
    but->tooltip="drag Me!";
    but->setLocation(Vector3f(xP2[0]+mainW+firstW-16,yP2[0]-16,0));
    but->scale=eckScale;
    but->color=eckColor;
    but->bTriggerWhileDragging=true;
    but->bResetAfterDrag=false;
    but->textureID="ecke";
    but->parent=this;
    but->setup();
    renderer->buttonList.push_back(but);

    but= new AssignButton;
    but->name="eckeB2";
    but->bDrawName=true;
    but->bDragable=true;
    but->tooltip="drag Me!";
    but->setLocation(Vector3f(xP2[1]+mainW+firstW-16,yP2[1]-16,0));
    but->scale=eckScale;
    but->color=eckColor;
    but->bTriggerWhileDragging=true;
    but->bResetAfterDrag=false;
    but->textureID="ecke";
    but->parent=this;
    but->setup();
    renderer->buttonList.push_back(but);

    but= new AssignButton;
    but->name="eckeB3";
    but->bDrawName=true;
    but->bDragable=true;
    but->tooltip="drag Me!";
    but->setLocation(Vector3f(xP2[2]+mainW+firstW-16,yP2[2]-16,0));
    but->scale=eckScale;
    but->color=eckColor;
    but->bTriggerWhileDragging=true;
    but->bResetAfterDrag=false;
    but->textureID="ecke";
    but->parent=this;
    but->setup();
    renderer->buttonList.push_back(but);

    but= new AssignButton;
    but->name="eckeB4";
    but->bDrawName=true;
    but->bDragable=true;
    but->tooltip="drag Me!";
    but->setLocation(Vector3f(xP2[3]+mainW+firstW-16,yP2[3]-16,0));
    but->scale=eckScale;
    but->color=eckColor;
    but->bTriggerWhileDragging=true;
    but->bResetAfterDrag=false;
    but->textureID="ecke";
    but->parent=this;
    but->setup();
    renderer->buttonList.push_back(but);


// camera Corners
    but= new AssignButton;
    but->name="ecke1c";
    but->bDrawName=true;
    but->bDragable=true;
    but->tooltip="drag Me!";
    but->setLocation(Vector3f(xC[0]+1000-16,yC[0]-16,0));
    but->scale=eckScale;
    but->color=eckColor;
    but->bTriggerWhileDragging=true;
    but->bResetAfterDrag=false;
    but->textureID="ecke";
    but->parent=this;
    but->setup();
    renderer->buttonList.push_back(but);

    but= new AssignButton;
    but->name="ecke2c";
    but->bDrawName=true;
    but->bDragable=true;
    but->tooltip="drag Me!";
    but->setLocation(Vector3f(xC[1]+1000-16,yC[1]-16,0));
    but->scale=eckScale;
    but->color=eckColor;
    but->bTriggerWhileDragging=true;
    but->bResetAfterDrag=false;
    but->textureID="ecke";
    but->parent=this;
    but->setup();
    renderer->buttonList.push_back(but);

    but= new AssignButton;
    but->name="ecke3c";
    but->bDrawName=true;
    but->bDragable=true;
    but->tooltip="drag Me!";
    but->setLocation(Vector3f(xC[2]+1000-16,yC[2]-16,0));
    but->scale=eckScale;
    but->color=eckColor;
    but->bTriggerWhileDragging=true;
    but->bResetAfterDrag=false;
    but->textureID="ecke";
    but->parent=this;
    but->setup();
    renderer->buttonList.push_back(but);

    but= new AssignButton;
    but->name="ecke4c";
    but->bDrawName=true;
    but->bDragable=true;
    but->tooltip="drag Me!";
    but->setLocation(Vector3f(xC[3]+1000-16,yC[3]-16,0));
    but->scale=eckScale;
    but->color=eckColor;
    but->bTriggerWhileDragging=true;
    but->bResetAfterDrag=false;
    but->textureID="ecke";
    but->parent=this;
    but->setup();
    renderer->buttonList.push_back(but);

}
