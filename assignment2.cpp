#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <fstream>
#include<math.h>
#include <time.h>
using namespace cv;
using namespace std;
struct point{
int image;
float wx;
float wy;
float wz;
float ww;
float ix;
float iy;
float iw;
};

class image
{
    public:
    int no;
    int no_of_points;
    vector<point> points;
};
int main( int argc, char** argv )
{
    while(1)
{

///open read and write files
    ifstream myfile;
    myfile.open("ImageDetails.txt", ios::in);
    vector<image> imagestack;
    if (myfile.is_open())
      {
        int f1,f2;

        for(int i=0;i<9;i++)
        {
            myfile >> f1 >> f2;
            image img;
            img.no=f1;
            img.no_of_points=f2;
            vector<point> dots;

            for(int i=0;i<f2;i++)
            {
              struct point p;
              myfile >> p.ix >> p.iy >> p.wx >> p.wy>> p.wz;
              p.image=f1;
              p.iw=1;
              p.ww=1;
              dots.push_back(p);
            }
            img.points=dots;
            imagestack.push_back(img);
        }
        myfile.close();
      }
else {cout << "error while reading file";return 0;}

///CHOOSE METHOD
    int methodno;
cout<<"What do you want to do? :\t1.DLT \t2.SVD \t3.RANSAC\t4.**Exit**\nPLease enter your choice:";
cin>>methodno;
    if(methodno==4){return 1;}

///CHOOSE IMAGE
    int imageno;
cout<<"which image do you want to select? 0 - 8  :";
cin>>imageno;

///MAKE FILENAMES
    stringstream imgstr;
    stringstream mthdstr;
    imgstr<<(imageno+1);
    mthdstr<<methodno;

    string outname="outofimage"+imgstr.str()+"method"+mthdstr.str()+".txt";
    string errorfilename="errorofimage"+imgstr.str()+"method"+mthdstr.str()+".txt";
    ofstream outfile;
    ofstream errorfile;
    outfile.open(outname.c_str(), ios::out);
    errorfile.open(errorfilename.c_str(), ios::out);

///OUTPUTS
    Mat Pcam;
    float cumul_error=0;
    vector<point> consensus_set;

///Parameters
    // no of entries in data set.
    int n=imagestack[imageno].points.size();
    //no of points in a sample
    int s=6;
    //max no_of_iterations
    int N=5000;//cout<<"no of iteraions :"<<endl;cin>>no_of_iterations;

    //inlier classifier threshold
    float tow=0.5;//cout<<"tow :"<<endl;cin>>tow;

    //inlier threshold_no
    int percent=80;//cout<<"\n enter threshold percentage :";cin>>percent;
    int threshold_no=(n*percent)/100;
///


///Points into Matrices.

Mat A;
    switch(methodno)
    {
     case 1:
         if(1){

        srand(time(NULL));
        RNG gener(rand());
        int randpoints[s];
        A=(Mat_<float>(2*s,2*s));
        float a[2*s][2*s];
        for(int i=0;i<s;i++)
        {
            randpoints[i]=gener.uniform(0,n);
            a[2*i][0]=imagestack[imageno].points[randpoints[i]].wx;
            a[2*i][1]=imagestack[imageno].points[randpoints[i]].wy;
            a[2*i][2]=imagestack[imageno].points[randpoints[i]].wz;
            a[2*i][3]=1;
            a[2*i][4]=0;
            a[2*i][5]=0;
            a[2*i][6]=0;
            a[2*i][7]=0;
            a[2*i][8]=-imagestack[imageno].points[randpoints[i]].ix*imagestack[imageno].points[randpoints[i]].wx;
            a[2*i][9]=-imagestack[imageno].points[randpoints[i]].ix*imagestack[imageno].points[randpoints[i]].wy;
            a[2*i][10]=-imagestack[imageno].points[randpoints[i]].ix*imagestack[imageno].points[randpoints[i]].wz;
            a[2*i][11]=-imagestack[imageno].points[randpoints[i]].ix;


            a[2*i+1][0]=0;
            a[2*i+1][1]=0;
            a[2*i+1][2]=0;
            a[2*i+1][3]=0;
            a[2*i+1][4]=imagestack[imageno].points[randpoints[i]].wx;
            a[2*i+1][5]=imagestack[imageno].points[randpoints[i]].wy;
            a[2*i+1][6]=imagestack[imageno].points[randpoints[i]].wz;
            a[2*i+1][7]=1;
            a[2*i+1][8]=-imagestack[imageno].points[randpoints[i]].iy*imagestack[imageno].points[randpoints[i]].wx;
            a[2*i+1][9]=-imagestack[imageno].points[randpoints[i]].iy*imagestack[imageno].points[randpoints[i]].wy;
            a[2*i+1][10]=-imagestack[imageno].points[randpoints[i]].iy*imagestack[imageno].points[randpoints[i]].wz;
            a[2*i+1][11]=-imagestack[imageno].points[randpoints[i]].iy;
        }
        A=Mat(2*s,2*s, CV_32FC1, a);
//cout<<A;
        Mat B=(Mat_<float>(2*s,1)<<0,0,0,0,0,0,0,0,0,0,0,0);
        Mat X;
        SVD::solveZ(A,X);
//cout<<endl<<"Psh... I dont have to use a calculator\n X is "<<X<<endl;
        Pcam=X.reshape(0,3);
 cout<< Pcam << endl;
         }
         break;
     case 2:
         if(1){
            A=(Mat_<float>(2*n,2*s));
            float a[2*n][2*s];
            //Make matrices from points
            for(int i=0;i<n;i++)
            {
                a[2*i][0]=imagestack[imageno].points[i].wx;
                a[2*i][1]=imagestack[imageno].points[i].wy;
                a[2*i][2]=imagestack[imageno].points[i].wz;
                a[2*i][3]=1;
                a[2*i][4]=0;
                a[2*i][5]=0;
                a[2*i][6]=0;
                a[2*i][7]=0;
                a[2*i][8]=-imagestack[imageno].points[i].ix*imagestack[imageno].points[i].wx;
                a[2*i][9]=-imagestack[imageno].points[i].ix*imagestack[imageno].points[i].wy;
                a[2*i][10]=-imagestack[imageno].points[i].ix*imagestack[imageno].points[i].wz;
                a[2*i][11]=-imagestack[imageno].points[i].ix;


                a[2*i+1][0]=0;
                a[2*i+1][1]=0;
                a[2*i+1][2]=0;
                a[2*i+1][3]=0;
                a[2*i+1][4]=imagestack[imageno].points[i].wx;
                a[2*i+1][5]=imagestack[imageno].points[i].wy;
                a[2*i+1][6]=imagestack[imageno].points[i].wz;
                a[2*i+1][7]=1;
                a[2*i+1][8]=-imagestack[imageno].points[i].iy*imagestack[imageno].points[i].wx;
                a[2*i+1][9]=-imagestack[imageno].points[i].iy*imagestack[imageno].points[i].wy;
                a[2*i+1][10]=-imagestack[imageno].points[i].iy*imagestack[imageno].points[i].wz;
                a[2*i+1][11]=-imagestack[imageno].points[i].iy;
            }
            A=Mat(2*n,2*s, CV_32FC1, a);
            //cout<<A;
            Mat B=(Mat_<float>(2*n,1)<<0,0,0,0,0,0,0,0,0,0,0,0);
            Mat X=(Mat_<float>(2*n,1)<<0,0,0,0,0,0,0,0,0,0,0,0);

            solve(A,B,X,DECOMP_SVD);
//cout<<"Xinsolve "<<X<<endl;
            SVD::solveZ(A,X);
//cout<<"Xinsolvez "<<X<<endl;
            SVD decomposer(A);
            Pcam=X.reshape(0,3);
	    cout << Pcam << endl;
//cout<<"\n\n\n"<<endl;
        }
         break;
     case 3:
         if(1){
            //imagestack[imageno].points.size();


            ///***RANSAC ALGORITHM***///

            A=(Mat_<float>(2*n,2*s));
            float a[2*n][2*s];
            int max_inliners=0;
cout<<"threshold_no of points is :"<<threshold_no<<endl;
            //initiate min_error=inf;
            float min_error=1000000;
            //best p=blank
            Mat best_p=(Mat_<float>(2*s,1));
            //consensus_set=blank;
            srand (time(NULL));
            RNG gener(rand());
            ///while iterations<k    %k=no of iterations
            //cout<<endl<<endl<<endl;
            int k=0;
            for(k=0 ; k<N ; k++ )
                {
                    ///-pick 6 random points of correspndance=points_set;
                    ///-make a set
                    int randpoints[s];
                    vector<point> set_of_points;
                    Mat A=(Mat_<float>(2*s,2*s));
                    for(int i=0;i<s;i++)
                        {
                            randpoints[i]=gener.uniform(0,n);
                            //cout<<"random number is  : "<<randpoints[i]<<endl;
                            set_of_points.push_back(imagestack[imageno].points[randpoints[i]]);
                            a[2*i][0]=imagestack[imageno].points[randpoints[i]].wx;
                            a[2*i][1]=imagestack[imageno].points[randpoints[i]].wy;
                            a[2*i][1]=imagestack[imageno].points[randpoints[i]].wy;
                            a[2*i][2]=imagestack[imageno].points[randpoints[i]].wz;
                            a[2*i][3]=1;
                            a[2*i][4]=0;
                            a[2*i][5]=0;
                            a[2*i][6]=0;
                            a[2*i][7]=0;
                            a[2*i][8]=-imagestack[imageno].points[randpoints[i]].ix*imagestack[imageno].points[randpoints[i]].wx;
                            a[2*i][9]=-imagestack[imageno].points[randpoints[i]].ix*imagestack[imageno].points[randpoints[i]].wy;
                            a[2*i][10]=-imagestack[imageno].points[randpoints[i]].ix*imagestack[imageno].points[randpoints[i]].wz;
                            a[2*i][11]=-imagestack[imageno].points[randpoints[i]].ix;


                            a[2*i+1][0]=0;
                            a[2*i+1][1]=0;
                            a[2*i+1][2]=0;
                            a[2*i+1][3]=0;
                            a[2*i+1][4]=imagestack[imageno].points[randpoints[i]].wx;
                            a[2*i+1][5]=imagestack[imageno].points[randpoints[i]].wy;
                            a[2*i+1][6]=imagestack[imageno].points[randpoints[i]].wz;
                            a[2*i+1][7]=1;
                            a[2*i+1][8]=-imagestack[imageno].points[randpoints[i]].iy*imagestack[imageno].points[randpoints[i]].wx;
                            a[2*i+1][9]=-imagestack[imageno].points[randpoints[i]].iy*imagestack[imageno].points[randpoints[i]].wy;
                            a[2*i+1][10]=-imagestack[imageno].points[randpoints[i]].iy*imagestack[imageno].points[randpoints[i]].wz;
                            a[2*i+1][11]=-imagestack[imageno].points[randpoints[i]].iy;
                        }
                    A=Mat(2*s,2*s, CV_32FC1, a);

                    ///-find p
                    Mat p;
                    SVD::solveZ(A,p);

                    ///-Find distances and error
                    float distances[n];
                    float error=0.0;
                    for(int j=0;j<n;j++)
                        {
                            point worldpoint=imagestack[imageno].points[j];
                            Mat X=(Mat_<float>(4,1)<<worldpoint.wx,worldpoint.wy,worldpoint.wz,1 );
                            Mat P=p.reshape(0,3);
                            Mat projectivepoint=P*X;


                            Mat projimg=(Mat_<float>(2,1)<<(projectivepoint.at<float>(0,0)/projectivepoint.at<float>(2,0)),
                                         (projectivepoint.at<float>(1,0)/projectivepoint.at<float>(2,0)));
                            Mat actimg=(Mat_<float>(2,1)<<imagestack[imageno].points[j].ix,imagestack[imageno].points[j].iy);

                            Mat disterr=(projimg-actimg);
                            float projerr=norm(disterr);
                            distances[j]=projerr;
                            error=error+projerr;

                        }

                    ///Find no of inliners and push to consensus_set
                    int no_of_inliners=0;
                    for(int i=0;i<n;i++)
                        {
                            if(distances[i]<tow){no_of_inliners+=1;set_of_points.push_back(imagestack[imageno].points[i]);}
                        }
//cout<<"no_of_inliners  : "<<no_of_inliners<<endl;

                    ///-if inliners = higher than threshol_no find error and compare
//cout<<"is no of inliners "<<no_of_inliners<<" > "<<threshold_no<<" ? "<<endl;
                    if (no_of_inliners>threshold_no)
                    {
//cout<<"Threshold crossed. Now behold!!!"<<endl;
                                best_p=p;
                                consensus_set=set_of_points;
                                cumul_error=error;
                                max_inliners=no_of_inliners;
                                break;
                    }
                    else
                    {
//cout<<"NO im sorry to say u disappointed me BUT HEYY!!"<<endl;
                        if (no_of_inliners>max_inliners)
                        {
//cout<<"we have a new winner"<<endl;

//cout<<"Yeee error is minimum"<<endl;
                                    best_p=p;
                                    consensus_set=set_of_points;
                                    cumul_error=error;
                                    max_inliners=no_of_inliners;
                        }
                    }
                    ///repeat loop

                }
            ///consensus_set to A and cumulative Pcam

            if(1)
             {
                   Mat A=(Mat_<float>(2*s,2*s));
                   float ap[2*consensus_set.size()][2*s];
                   ///and make A,set_of_points.
                   for(int i=0;i<consensus_set.size();i++)
                        {
                ap[2*i][0]=imagestack[imageno].points[i].wx;
                ap[2*i][1]=imagestack[imageno].points[i].wy;
                ap[2*i][2]=imagestack[imageno].points[i].wz;
                ap[2*i][3]=1;
                ap[2*i][4]=0;
                ap[2*i][5]=0;
                ap[2*i][6]=0;
                ap[2*i][7]=0;
                ap[2*i][8]=-imagestack[imageno].points[i].ix*imagestack[imageno].points[i].wx;
                ap[2*i][9]=-imagestack[imageno].points[i].ix*imagestack[imageno].points[i].wy;
                ap[2*i][10]=-imagestack[imageno].points[i].ix*imagestack[imageno].points[i].wz;
                ap[2*i][11]=-imagestack[imageno].points[i].ix;


                ap[2*i+1][0]=0;
                ap[2*i+1][1]=0;
                ap[2*i+1][2]=0;
                ap[2*i+1][3]=0;
                ap[2*i+1][4]=imagestack[imageno].points[i].wx;
                ap[2*i+1][5]=imagestack[imageno].points[i].wy;
                ap[2*i+1][6]=imagestack[imageno].points[i].wz;
                ap[2*i+1][7]=1;
                ap[2*i+1][8]=-imagestack[imageno].points[i].iy*imagestack[imageno].points[i].wx;
                ap[2*i+1][9]=-imagestack[imageno].points[i].iy*imagestack[imageno].points[i].wy;
                ap[2*i+1][10]=-imagestack[imageno].points[i].iy*imagestack[imageno].points[i].wz;
                ap[2*i+1][11]=-imagestack[imageno].points[i].iy;
                        }
                   Mat X=(Mat_<float>(2*n,1)<<0,0,0,0,0,0,0,0,0,0,0,0);
                   A=Mat(2*s,2*s, CV_32FC1, a);
                   SVD::solveZ(A,X);
                   Pcam=X.reshape(0,3);
            }

//cout<<"\n\n\nbest_p is  :"<<best_p<<"\n\n\ncumul_error   :"<<cumul_error<<endl;
            Pcam=best_p.reshape(0,3);
//cout<<"Best Pcam is  :"<<Pcam<<endl<<endl<<endl;
//cout<<"cumul_error is  :"<<cumul_error<<endl<<endl;

cout<<"inliners_max  ="<<max_inliners;
cout<<"  total iterations = :"<<k<<endl;
         }
         break;
     case 4:
        return 0;
    }

outfile<<"image :"<<imageno+1<<"method :"<<methodno<<endl;
outfile<<"P ="<<Pcam<<endl<<endl;
if(methodno==3)
{
outfile<<"  inlier threshold  :"<<tow<<"  no_of_inliers :"<<threshold_no<<endl;
outfile<<"error = "<<(cumul_error*1.0)/n<<endl;
outfile<<"Ground data       :       projected points"<<endl;
    cumul_error=0;
}

errorfile<<"image :"<<imageno+1<<"method :"<<methodno<<endl;

///FINAL REPORT
    for(int i=0;i<n;i++)
        {
            Mat X=(Mat_<float>(4,1)<<imagestack[imageno].points[i].wx,imagestack[imageno].points[i].wy,
                   imagestack[imageno].points[i].wz,1);
            Mat proj=Pcam*X;
            Mat projimg=(Mat_<float>(2,1)<<(proj.at<float>(0,0)/proj.at<float>(2,0)),(proj.at<float>(1,0)/proj.at<float>(2,0)));
            Mat actimg=(Mat_<float>(2,1)<<imagestack[imageno].points[i].ix,imagestack[imageno].points[i].iy);
            Mat disterr=(projimg-actimg);
            float projerr=norm(disterr);
            cumul_error=cumul_error+projerr;

errorfile<<projerr<<endl;
//errorfile<<"cumul_avg_error is :"<<(cumul_error*1.0)/n<<endl;

outfile<<i<<". ["<<imagestack[imageno].points[i].ix<<" "<<imagestack[imageno].points[i].iy<<"]  "
                   <<projimg<<"  error is : "<<projerr<<endl;
        }
errorfile<<"cumul_avg_error is :"<<(cumul_error*1.0)/n<<endl;

cout<<(cumul_error*1.0)/n<<endl;
    errorfile.close();
    outfile.close();
}

}


