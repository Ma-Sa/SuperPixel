#include <time.h>
#include <stdio.h>
#include <pangolin/pangolin.h>
#include <pangolin/timer.h>
#include <HAL/Camera/CameraDevice.h>
#include <HAL/IMU/IMUDevice.h>
#include <HAL/Posys/PosysDevice.h>

#include<HAL/Utils/GetPot>
#include<HAL/Utils/TicToc.h>

#include<PbMsgs/Logger.h>
#include<PbMsgs/Matrix.h>

#include "FastImgSeg.h"
#include "cudaDefines.h"
#include "cudaImgTrans.h"
#include "cudaSegEngine.h"
#include "cudaSegSLIC.h"
#include "cudaUtil.h"

#include "cv.h"
#include "highgui.h"

using namespace std;

bool        g_bLog      = false;
pb::Logger& g_Logger    = pb::Logger::GetInstance();

pangolin::DataLog g_PlotLogAccel;
pangolin::DataLog g_PlotLogGryo;
pangolin::DataLog g_PlotLogMag;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void IMU_Handler(pb::ImuMsg& IMUdata)
{
    if( g_bLog ) {
        pb::Msg pbMsg;
        pbMsg.set_timestamp( hal::Tic() );
        pbMsg.mutable_imu()->Swap(&IMUdata);
        g_Logger.LogMessage(pbMsg);
    }
//    const pb::VectorMsg& pbVec = IMUdata.accel();
//    printf("X: %5f    Y: %5f     Z: %5f\r",pbVec.data(0),pbVec.data(1),pbVec.data(2));
    if( IMUdata.has_accel() ) {
        g_PlotLogAccel.Log( IMUdata.accel().data(0), IMUdata.accel().data(1), IMUdata.accel().data(2) );
    }
    if( IMUdata.has_gyro() ) {
        g_PlotLogGryo.Log( IMUdata.gyro().data(0), IMUdata.gyro().data(1), IMUdata.gyro().data(2) );
    }
    if( IMUdata.has_mag() ) {
        g_PlotLogMag.Log( IMUdata.mag().data(0), IMUdata.mag().data(1), IMUdata.mag().data(2) );
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Posys_Handler(pb::PoseMsg& PoseData)
{
    
    pb::VectorMsg pbVec = PoseData.pose();
    if( PoseData.id() == 0 ) {
        printf("%4f   %4f   %4f -- %4f   %4f   %4f   %4f\r",pbVec.data(0),pbVec.data(1),pbVec.data(2),
           pbVec.data(3),pbVec.data(4),pbVec.data(5),pbVec.data(6));
        fflush(stdout);
    }
    

    if( g_bLog ) {
        pb::Msg pbMsg;
        pbMsg.set_timestamp( hal::Tic() );
        pbMsg.mutable_pose()->Swap(&PoseData);
        g_Logger.LogMessage(pbMsg);
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main( int argc, char* argv[] )
{
    GetPot clArgs(argc,argv);

    ///-------------------- CAMERA INIT (Optional)

    std::string sCam = clArgs.follow("", "-cam" );
    const bool bHaveCam = !sCam.empty();

    // N cameras, each w*h in dimension, greyscale
    size_t nNumChannels = 0;
    size_t nImgWidth = 0;
    size_t nImgHeight = 0;

    hal::Camera theCam;
    pb::ImageArray vImgs;
    if(bHaveCam) {
        theCam = hal::Camera(sCam);
        nNumChannels = theCam.NumChannels();
        nImgWidth = theCam.Width();
        nImgHeight = theCam.Height();

        std::cout << "- Opening camera with " << nNumChannels << " channel(s)." << std::endl;
        for(size_t ii=0; ii<nNumChannels; ++ii) {
            std::cout << "  " << theCam.Width(ii) << "x" << theCam.Height(ii) << std::endl;
        }
    }



    ///-------------------- WINDOW INIT

    // Setup OpenGL Display (based on GLUT)
    pangolin::CreateWindowAndBind(__FILE__,nNumChannels*nImgWidth,nImgHeight *  2.0);

    glPixelStorei(GL_PACK_ALIGNMENT,1);
    glPixelStorei(GL_UNPACK_ALIGNMENT,1);
    pangolin::GlTexture glTex;

    // Create Smart viewports for each camera image that preserve aspect
    pangolin::View& cameraView = pangolin::CreateDisplay().SetLayout(pangolin::LayoutEqual);
    for(size_t ii=0; ii < nNumChannels*2; ++ii ) {
        cameraView.AddDisplay(pangolin::CreateDisplay().SetAspect((double)nImgWidth/nImgHeight));
    }


    bool bRun = true;
    bool bStep = false;
    unsigned long nFrame=0;

    pangolin::RegisterKeyPressCallback(pangolin::PANGO_SPECIAL + pangolin::PANGO_KEY_RIGHT, [&bStep](){bStep=true;} );
    pangolin::RegisterKeyPressCallback(' ', [&](){bRun = !bRun;} );
    pangolin::RegisterKeyPressCallback('l', [&](){ g_bLog = !g_bLog; nFrame = 0; } );

    pangolin::Timer theTimer;

      cvNamedWindow("myWindow", CV_WINDOW_AUTOSIZE);




    for(; !pangolin::ShouldQuit(); nFrame++)
    {
        const bool bGo = bRun || pangolin::Pushed(bStep);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glColor4f(1.0f,1.0f,1.0f,1.0f);

        if(bGo && nNumChannels) {
            if( !theCam.Capture(vImgs) ) {
                bRun = false;
            }

#ifdef HAVE_GLUT
            if(nFrame%30 == 0) {
                char buffer[1024];
                sprintf(buffer,"SuperPixel (FPS: %f)", 30.0 / theTimer.Elapsed_s() );
                glutSetWindowTitle(buffer);
                theTimer.Reset();
            }
#endif
        }

        if(!glTex.tid && nNumChannels) {
            // Only initialise now we know format.
            glTex.Reinitialise(
                nImgWidth, nImgHeight, vImgs[0].Format(), true, 0,
                vImgs[0].Format(),vImgs[0].Type(), 0
            );
        }

        for(size_t ii=0; ii<nNumChannels; ++ii ) {
            cameraView[ii].Activate();
            glTex.Upload( vImgs[ii].data(), vImgs[ii].Format(), vImgs[ii].Type() );
            glTex.RenderToViewportFlipY();

            if(ii%2==0){

             cv::Mat myImage = cv::Mat (vImgs[ii]);
             IplImage temp= myImage;
             IplImage* frame = &temp;

             cvShowImage("myWindow", frame);
             cvSaveImage("test.jpg",frame);
             FastImgSeg* mySeg=new FastImgSeg();
             mySeg->initializeFastSeg(frame->width,frame->height,500);

             unsigned char* imgBuffer=(unsigned char*)malloc(frame->width*frame->height*sizeof(unsigned char)*4);
             memset(imgBuffer,0,frame->width*frame->height*sizeof(unsigned char)*4);


                    // gSLIC currently only support 4-dimensional image
                    for (int i=0;i<frame->height;i++)
                    {
                        for (int j=0;j<frame->width;j++)
                        {
                            int bufIdx=(i*frame->width+j)*4;

                            imgBuffer[bufIdx]=CV_IMAGE_ELEM(frame,unsigned char,i,j*3);
                            imgBuffer[bufIdx+1]=CV_IMAGE_ELEM(frame,unsigned char,i,j*3+1);
                            imgBuffer[bufIdx+2]=CV_IMAGE_ELEM(frame,unsigned char,i,j*3+2);
                        }
                    }

                    mySeg->LoadImg(imgBuffer);
                    mySeg->DoSegmentation(XYZ_SLIC,0.3);
                    mySeg->Tool_GetMarkedImg();

                    for (int i=0;i<frame->height;i++)
                    {
                        for (int j=0;j<frame->width;j++)
                        {
                            int bufIdx=(i*frame->width+j)*4;

                            CV_IMAGE_ELEM(frame,unsigned char,i,j*3)=mySeg->markedImg[bufIdx];
                            CV_IMAGE_ELEM(frame,unsigned char,i,j*3+1)=mySeg->markedImg[bufIdx+1];
                            CV_IMAGE_ELEM(frame,unsigned char,i,j*3+2)=mySeg->markedImg[bufIdx+2];
                        }
                    }
                    cvShowImage("myWindow", frame);
                    if( cvWaitKey(10) == 27 )
                        break;

            }
        }


        if(g_bLog && bRun) {
            pb::Msg pbMsg;
            pbMsg.set_timestamp( hal::Tic() );
            pbMsg.mutable_camera()->Swap(&vImgs.Ref());
            g_Logger.LogMessage(pbMsg);

            // draw red circle on bottom left corner for visual cue
            if( ! ((nFrame / 30) %2) ) {
                cameraView.ActivatePixelOrthographic();
                glPushAttrib(GL_ENABLE_BIT);
                glDisable(GL_LIGHTING);
                glDisable(GL_DEPTH_TEST);
                glColor3ub( 255, 0, 0 );
                pangolin::glDrawCircle(20,20,7);
                glPopAttrib();
            }
        }

        pangolin::FinishFrame();
    }
}




