#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <list>
#include <algorithm>
#include <sstream>
#include <string>

using namespace cv;
using namespace std;

typedef cv::Point3_<uint8_t> Pixel;

bool color_pool[31*31*31];

void init_color_pool()
{
    for (int r=1; r<31*31*31; r++) color_pool[r]=true; 

    for (int r=1; r<32; r++)
    for (int g=1; g<32; g++)
    for (int b=1; b<32; b++)
    color_pool[(r << 10) + (g << 5) +b]=false;

}

cv::Scalar get_colour_from_pool()
{
    static int seed=21312;

    while (color_pool[seed]==true)
    {
        seed=seed+7236;
        if (seed>31*31*31)
        {
            seed-=31*31*31;
        }
    }
    
    color_pool[seed]=true;
    int r=(seed & 31744) >> 7;
    int g=(seed & 992) >> 2;
    int b=(seed & 31) << 3;
    return CV_RGB(r,g,b);
}

void ScanImageAndReduceC(Mat &I,Mat &image)
{
    // accept only char type matrices
    CV_Assert(I.depth() == CV_8U);
    
    int nRows = I.rows;
    int nCols = I.cols;
    int channels = I.channels() ;

    I.copyTo(image);
    init_color_pool();

    //Colourize tracks with different colours
    for (int r =0; r<nRows; r++)
    {   
        for (int c =0; c<nCols; c++)
        {   Vec3b colour = image.at<Vec3b>(Point(c, r));
            
            if(colour.val[0]==255 && colour.val[1]==255 && colour.val[2]==255)
            {
                floodFill(image,Point(c,r), get_colour_from_pool() );

            }
        }
        
    }

    namedWindow( "Heady", WINDOW_GUI_NORMAL ); // Create a window for display.
    resizeWindow("Heady", 1024, 1237);
    imshow("Heady",image);
    waitKey(1);

    // Dilate with colours
    typedef unsigned char byte;
    bool change=true;
    byte rr[9],gg[9],bb[9];
    int tb[32768];
    char cr,cg,cb;
    int index,tmaxc;

    Mat dimage[3];
    Mat newimage;
    
    while (change)
    {
        change=false;

        image.copyTo(newimage);

        for (int r=1; r<(nRows-1); r++)
        {   
            for (int c=1; c<(nCols-1); c++)
            {
                if ((uchar) image.at<Vec3b>(r,c)[2]==0)
                {
                    Mat roi = image(Rect(c-1,r-1,3,3));
                    split(roi,dimage);

                    memcpy(bb,dimage[2].data, 9*sizeof(byte));

                    bool edge=false;
                    for (int lll=0; lll<9; lll++) if ((int)bb[lll]>0) {edge=true;} 
                    
                    if (edge)
                    {    
                        memcpy(rr,dimage[0].data, 9*sizeof(byte));
                        memcpy(gg,dimage[1].data, 9*sizeof(byte));
                        memset(tb,0,32768*sizeof(int));

                        std::vector<int> colour_list = {};
                        
                        for (int l=0; l<9; l++)
                        {
                            if (l!=4)
                            {
                                index = ((rr[l] & 248) << 7) | ((gg[l] & 248) << 2) | (bb[l] >> 3);

                                tb[index]=tb[index]+1;
                                
                                if (std::find(std::begin(colour_list), std::end(colour_list), index) == std::end(colour_list))
                                {
                                    colour_list.insert(colour_list.begin(),index);
                                }
                                
                            }
                        }

                        tb[0]=0; // which is not background
                        index=0;
    
                        // Find colour which neighbours most pixels

                        for (std::vector<int>::iterator it = colour_list.begin() ; it != colour_list.end(); ++it)
                        {
                            //cout << index << " " << tb[index] << *it << tb[*it] << "\n";
                            if (tb[index]<tb[*it]) { index=*it;}
                            //cout << index << " " << tb[index] << *it << tb[*it] << "\n\n";
                        }


                        if ((int) tb[index]>0)
                        {   
                            /*cout << (int)tb[index] << " " << (int) index << ":";
                            for (int u=0; u<9; u++) cout << (int)rr[u] << (int)bb[u]  << (int)gg[u] << " ";
                                cout <<" \n";*/
                            cr=((index & 31744) >> 7);
                            cg=((index & 992) >> 2);
                            cb=((index & 31) << 3);

                            newimage.at<Vec3b>(Point(c, r))=Vec3b(cr,cg,cb);
                        }

                        change=true;
                    } 
                }
            }

        }
        namedWindow( "Heady", WINDOW_GUI_NORMAL ); // Create a window for display.
        resizeWindow("Heady", 1024, 1237);
        imshow("Heady",newimage);
        waitKey(1);

        newimage.copyTo(image);
        cout << ".\n";
    }
    

};

void DrillHoles(Mat &image)
{
    std::ifstream infile("drill.dcf");
    std::string rline;
    int start;
    int vector[5];

    while (std::getline(infile, rline))
    {
        start=0;
        for (int t=0; t<5; t++) 
        {
            std::size_t found = rline.find(",");
            vector[t]=std::stoi(rline.substr(start,found));
            rline = rline.substr(found+1);
        }

        if ((vector[0]==vector[2]) && (vector[1]==vector[3]))
        {
            circle(image, Point(vector[0],vector[1]), vector[4], Vec3b(0,0,0), -1);
        } else
        {
            line(image, Point(vector[0],vector[1]), Point(vector[2],vector[3]), Vec3b(0,0,0), vector[4]);
        }
        

    }

}

int main( int argc, char** argv )
{
    bool file_is_ready = false;

    Mat image,city;

    if (!file_is_ready)
    {
        String imageName( "map.png" ); // by default
        if( argc > 1)
        {
            imageName = argv[1];
        }
        
        image = imread( samples::findFile( imageName ), IMREAD_COLOR ); // Read the file
        if( image.empty() )                      // Check for invalid input
        {
            cout <<  "Could not open or find the image" << std::endl ;
            return -1;
        }
        
        city = Mat::zeros(Size(image.cols,image.rows),CV_8UC3);
        cout << city.size;

        ScanImageAndReduceC(image,city);
        imwrite( "cpp_image.png", city );
    }
    else
    {
        city = imread("cpp_image.png");
    }

    DrillHoles(city);

    Mat detected_edges;
    Canny( city, detected_edges, 50, 150, 7 );
    
    Mat element = getStructuringElement( MORPH_RECT,Size( 2*5 + 1, 2*5+1 ),Point( 5, 5 ) );
    dilate( detected_edges, detected_edges, element );
    erode( detected_edges, detected_edges, element );

    imwrite( "aout.png", detected_edges );      
    //contours = find_contours(edged, 240,fully_connected='high',positive_orientation='low')

    imshow( "Display window", detected_edges );                // Show our image inside it.
    waitKey(20000);
    return 0;
}
