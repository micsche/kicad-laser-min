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

using namespace std;
using namespace cv;

int pxmm = 30;
float linesegment[50000][5];
unsigned int iline[50000][5];

unsigned int linesegpos=0;
float boardminx=9.0e+9,boardminy=9.0e+9,boardmaxx=-9.0e+9,boardmaxy=-9.0e+9;
unsigned image_height,image_width;

void get_two_vals(std::string line, float *x, float *y)
{
    size_t pos =0;
    std::string token;

    line.erase(remove(line.begin(), line.end(), ')'), line.end());

    int posit=0;
    while ((pos = line.find(" ")) != string::npos)
    {   token = line.substr(0,pos);
        //cout << token << ",";
        line.erase(0,pos + 1);
        if (posit==1)
        {
            *x=std::stof(token);
        } else if (posit==2)
        {
            *y=std::stof(token);
        }
        posit++;
    }
    //cout << endl;
}

void get_one_valf(std::string line, float *width)
{   size_t pos =0;
    int posit=0;
    std::string token;
    
    line.erase(remove(line.begin(), line.end(), ')'), line.end());

    while ((pos = line.find(" ")) != string::npos)
    {   token = line.substr(0,pos);
        //cout << token << ",";
        line.erase(0,pos + 1);
        if (posit==1)
        {
            *width=std::stof(token);
        } 
        posit++;
    }
}

void get_one_vals(std::string line, std::string *value)
{   size_t pos =0;
    int posit=0;
    std::string token;
    
    line.erase(remove(line.begin(), line.end(), ')'), line.end());

    while ((pos = line.find(" ")) != string::npos)
    {   token = line.substr(0,pos);
        //cout << token << ",";
        line.erase(0,pos + 1);
        if (posit==1)
        {
            *value=token;
        } 
        posit++;
    }
}


int readkicad()
{   std::string line;
    std::string layer;
    float x1,y1,x2,y2,width;

    ifstream myfile ("simplez80-mobo.kicad_pcb");
    if (myfile.is_open())
    {
        while (! myfile.eof() )
        {
            getline (myfile,line);
            if (line.find("segment")!=string::npos)
            {
                size_t pos =0;
                std::string token;
                while ((pos = line.find("(")) != string::npos)
                {
                    token = line.substr(0,pos);

                    if (token.rfind("start",0) == 0)
                    {
                        get_two_vals(token,&x1,&y1);
                    }
                    if (token.rfind("end",0) == 0)
                    {
                        get_two_vals(token,&x2,&y2);
                    }
                    if (token.rfind("width",0) == 0)
                    {
                        get_one_valf(token,&width);
                    }
                    if (token.rfind("layer",0) == 0)
                    {
                        get_one_vals(token,&layer);

                        if (layer == "B.Cu")

                        {
                            linesegment[linesegpos][0]=x1;
                            linesegment[linesegpos][1]=y1;
                            linesegment[linesegpos][2]=x2;
                            linesegment[linesegpos][3]=y2;
                            linesegment[linesegpos][4]=width;

                            boardminx=min(x1,boardminx);
                            boardminx=min(x2,boardminx);

                            boardminy=min(y1,boardminy);
                            boardminy=min(y2,boardminy);

                            boardmaxx=max(x1,boardmaxx);
                            boardmaxx=max(x2,boardmaxx);

                            boardmaxy=max(y1,boardmaxy);
                            boardmaxy=max(y2,boardmaxy);

                            linesegpos++;
                        }
                    }
                    
                    line.erase(0,pos + 1);
                }
                
            }
            
        }
        myfile.close();
    }
    else cout << "Unable to open file"; 
    boardmaxx+=2;
    boardminx-=2;
    boardmaxy+=2;
    boardminy-=2;
    image_width = int(boardmaxx-boardminx)*pxmm;
    image_height = int(boardmaxy-boardminy)*pxmm;



    for (unsigned int li=0; li<linesegpos; li++)
    {
        iline[li][0]=int((linesegment[li][0]-boardminx)*pxmm);
        iline[li][2]=int((linesegment[li][2]-boardminx)*pxmm);

        iline[li][1]=int((linesegment[li][1]-boardminy)*pxmm);
        iline[li][3]=int((linesegment[li][3]-boardminy)*pxmm);

        iline[li][4]=int((linesegment[li][4])*pxmm);
    }
    cout << linesegpos << " segments" << endl;
    cout << "Board :" << image_width << "x" << image_height << endl;

  return 0;
}

void showpic()
{
    Mat city = Mat::zeros(Size(image_width,image_height),CV_8UC3);

    for (int i=0; i<linesegpos; i++)
    {
        line(city,Point2d(iline[i][0],iline[i][1]),Point2d(iline[i][2],iline[i][3]),Vec3b(255,255,255),iline[i][4]);
    }

    imwrite( "mapout.2.png", city );
}

int main()
{
    readkicad();
    showpic();
    return 0;
}

