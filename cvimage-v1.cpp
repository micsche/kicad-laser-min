#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
using namespace cv;
using namespace std;

typedef cv::Point3_<uint8_t> Pixel;

void ScanImageAndReduceC(Mat &I,Mat &image)
{
    // accept only char type matrices
    CV_Assert(I.depth() == CV_8U);
    
    int nRows = I.rows;
    int nCols = I.cols;
    int channels = I.channels() ;

    I.copyTo(image);

    //Colourize tracks with different colours

    int cR=30;
    int cG=30;
    int cB=30;

    for (int r =0; r<nRows; r++)
    {   

        for (int c =0; c<nCols; c++)
        {   Vec3b colour = image.at<Vec3b>(Point(c, r));
            
            if(colour.val[0]==255 && colour.val[1]==255 && colour.val[2]==255)
            {
                floodFill(image,Point(c,r),CV_RGB(cR,cG,cB) );

                cR=cR+32;
                if (cR>255)
                {
                    cG=cG+32;
                    cR=30;

                    if (cG>255)
                    {
                        cG=30;
                        cB=cB+32;
                    }

                }
            }
        }
        
    }

    // Dilate with colours
    typedef unsigned char byte;
    bool change=true;
    byte rr[9],gg[9],bb[9];
    char tb[512],cr,cg,cb;
    int index;

    Mat dimage[3];
    Mat newimage;
    

    while (change)
    {
        change=false;

        image.copyTo(newimage);

        for (int r=1; r<(nRows-1); r++)
        {   //cout << "\r" << to_string(r) << "/" << to_string(nRows) << endl;

            for (int c=1; c<(nCols-1); c++)
            {
                Mat roi = image(Rect(c-1,r-1,3,3));
                
                split(roi,dimage);
                memcpy(rr,dimage[0].data, 9*sizeof(byte));
                memcpy(gg,dimage[1].data, 9*sizeof(byte));
                memcpy(bb,dimage[2].data, 9*sizeof(byte));

                memset(tb,512,0);

                if ((char)rr[4]==0)
                {
                    for (int l=0; l<9; l++)
                    {
                        if (l!=4)
                        {
                            index = (int((rr[l]-30) >> 5) << 6)+(int((gg[l]-30) >> 5) << 3) + int((bb[l]-30) >> 5);
                            tb[index]=tb[index]+1;

                        }
                    }

                    index=0;
                    for (int l=0; l<512; l++) 
                    {
                        if (tb[l]>tb[index]) index=l;
                    }

                    cr=((index >> 6) << 5)+30;
                    cg=((index & 0x38) << 2) + 30;
                    cb=((index & 0x7) << 5) + 30;

                    newimage.at<Vec3b>(Point(c, r))=Vec3b(cr,cg,cb);

                    change=true;
                } 
            }

        }
        newimage.copyTo(image);
        cout << ".\n";
    }
    

};

int main( int argc, char** argv )
{
    String imageName( "map.png" ); // by default
    if( argc > 1)
    {
        imageName = argv[1];
    }
    Mat image,city;
    image = imread( samples::findFile( imageName ), IMREAD_COLOR ); // Read the file
    if( image.empty() )                      // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    
    city = Mat::zeros(Size(image.cols,image.rows),CV_8UC3);
    cout << city.size;
    ScanImageAndReduceC(image,city);

    namedWindow( "Display window", WINDOW_GUI_NORMAL ); // Create a window for display.
    resizeWindow("Display window", 1024, 1237);
    imshow( "Display window", city );                // Show our image inside it.
    waitKey(10000);
    return 0;
}
