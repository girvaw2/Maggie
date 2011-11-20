#include "face_tracking.h"
#include <iostream>

face_tracking::face_tracking ()
{
    char      *filename = "/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_frontalface_alt.xml";
 
    /* load the classifier
       note that I put the file in the same directory with
       this code */
    cascade = ( CvHaarClassifierCascade* )cvLoad( filename, 0, 0, 0 );
 
    /* setup memory buffer; needed by the face detector */
    storage = cvCreateMemStorage( 0 );
}

face_tracking::~face_tracking()
{
    cvReleaseHaarClassifierCascade( &cascade );
    cvReleaseMemStorage( &storage );
}
 
void face_tracking::detectFaces(sensor_msgs::Image image_in)
{
    int i;
       
    if (image_in.width == 0)
      return;
    //sensor_msgs::CvBridge bridge;
    // IplImage* img  = cv_bridge::imgMsgToCv(image_in, "bgr8");
     //IplImage* img = cv_bridge::toCvCopy(image_in, enc::BGR8);
    std::cin.get(); 
    
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image_in, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

//     IplImage* img  = &(cv_ptr->image);
    

    /* detect faces */
//     CvSeq *faces = cvHaarDetectObjects(
//             (cv_ptr->image).ptr(),
//             cascade,
//             storage,
//             1.1,
//             3,
//             0 /*CV_HAAR_DO_CANNY_PRUNNING*/,
//             cvSize( 40, 40 ) );
 
    /* for each face found, draw a red box */
//     for( i = 0 ; i < ( faces ? faces->total : 0 ) ; i++ ) {
//         CvRect *r = ( CvRect* )cvGetSeqElem( faces, i );
//         cvRectangle( img,
//                      cvPoint( r->x, r->y ),
//                      cvPoint( r->x + r->width, r->y + r->height ),
//                      CV_RGB( 255, 0, 0 ), 1, 8, 0 );
//     }
 
    /* display video */
    cvShowImage( "video", &(cv_ptr));
}


