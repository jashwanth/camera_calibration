#include<iostream>
#include<stdio.h>
#include<math.h>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/objdetect/objdetect.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<iomanip>
#include<string>
#include<sstream>
#include<stdlib.h>
#define threshold (double)1.0
#define prob (double)0.9
using namespace std;
using namespace cv;
Mat basic_lse(vector<Point3f> objects,vector<Point2f> images,Mat *K,Mat *R,Mat *T,int no_points){
	int i,j,k;
	double mat[12][12];
	for (i=0;i<6;i++){
		j=rand()%no_points;
		float a,b,x,y,z;
		a=images[j].x;
		b=images[j].y;
		x=objects[j].x;
		y=objects[j].y;
		z=objects[j].z;
		k=2*i;
		mat[k][0]=x,mat[k][1]=y,mat[k][2]=z,mat[k][3]=1;
		for (j=4;j<8;j++) mat[k][j]=0;
		mat[k][8]=(-1)*a*x;
		mat[k][9]=(-1)*a*y;
		mat[k][10]=(-1)*a*z;
		mat[k][11]=(-1)*a;
		k++;
		mat[k][4]=x,mat[k][5]=y,mat[k][6]=z,mat[k][7]=1;
		for (j=0;j<4;j++) mat[k][j]=0;
		mat[k][8]=(-1)*b*x;
		mat[k][9]=(-1)*b*y;
		mat[k][10]=(-1)*b*z;
		mat[k][11]=(-1)*b;
	}
	Mat A,P;
	A=Mat(12,12,CV_64F,mat);
	SVD::solveZ(A,P);
	P=P.reshape(0,3);
//	cout<<P.size()<<endl;
	decomposeProjectionMatrix(P,*K,*R,*T);
	return P;
}
Mat my_svd(vector<Point3f> objects,vector<Point2f> images,Mat *K,Mat *R,Mat *T,int no_points){
	int i,j,k;
	double mat[2*no_points][12];
	for (i=0;i<no_points;i++){
		float a,b,x,y,z;
		a=images[i].x;
		b=images[i].y;
		x=objects[i].x;
		y=objects[i].y;
		z=objects[i].z;
		k=2*i;
		mat[k][0]=x,mat[k][1]=y,mat[k][2]=z,mat[k][3]=1;
		for (j=4;j<8;j++) mat[k][j]=0;
		mat[k][8]=(-1)*a*x;
		mat[k][9]=(-1)*a*y;
		mat[k][10]=(-1)*a*z;
		mat[k][11]=(-1)*a;
		k++;
		mat[k][4]=x,mat[k][5]=y,mat[k][6]=z,mat[k][7]=1;
		for (j=0;j<4;j++) mat[k][j]=0;
		mat[k][8]=(-1)*b*x;
		mat[k][9]=(-1)*b*y;
		mat[k][10]=(-1)*b*z;
		mat[k][11]=(-1)*b;
	}
	Mat A,P;
	A=Mat(2*no_points,12,CV_64F,mat);
	SVD::solveZ(A,P);
	P=P.reshape(0,3);
	decomposeProjectionMatrix(P,*K,*R,*T);
	return P;
}
Mat my_ransac(vector<Point3f> objects,vector<Point2f> images,Mat *K,Mat *R,Mat *T,int no_points){
	int i,j,k,ans=0;
	Mat m;
	long double w=0;
	int N=INT_MAX,n=0;
	while (1){
		Mat K_temp,R_temp,T_temp,P;
		P=basic_lse(objects,images,&K_temp,&R_temp,&T_temp,no_points);
		int no_inliers=0;
		for (i=0;i<no_points;i++){
			double x[3],X[]={objects[i].x,objects[i].y,objects[i].z,1};
			for (j=0;j<3;j++){
				x[j]=0;
				for (k=0;k<4;k++){
					x[j]+=X[k]*P.at<double>(j,k);
				}
			}
			x[0]=x[0]/x[2],x[1]=x[1]/x[2];
			if ((images[i].x-x[0])*(images[i].x-x[0])+(images[i].y-x[1])*(images[i].y-x[1])<=threshold*threshold){
				no_inliers++;
			}
		}
		if (no_inliers>ans){
			ans=no_inliers;
			K_temp.copyTo(*K);
			R_temp.copyTo(*R);
			T_temp.copyTo(*T);
			P.copyTo(m);
		}
//		printf("%d\n",no_inliers);
		n++;
		w=(no_inliers*1.0)/no_points;
		w=log(1-pow(w,6));
		if (w!=0) N=log(1-prob)/w;
		if (abs(n-N)<=3) break;
	}
//	printf("%d %d %d\n",ans,n,N);
	return m;
}
void in_built(vector<Point3f> O,vector<Point2f> I,Mat *K,Mat *R,Mat *T,int no_points){
	vector<vector<Point3f> > objects;
	vector<vector<Point2f> > images;
	objects.push_back(O);
	images.push_back(I);
	Mat dist_coeffs=Mat::zeros(8,1,CV_64F);
	*K=initCameraMatrix2D(objects,images,Size(640,480),0);
	solvePnP(O,I,*K,dist_coeffs,*R,*T,false,CV_ITERATIVE);
}
double my_projection_error1(vector<Point3f> objects,vector<Point2f> images,Mat P,int no_points){
	int i,j,k;
	double mean=0;
	for (i=0;i<no_points;i++){
		double x[3],X[]={objects[i].x,objects[i].y,objects[i].z,1};
		for (j=0;j<3;j++){
			x[j]=0;
			for (k=0;k<4;k++){
				x[j]+=X[k]*P.at<double>(j,k);
			}
		}
		x[0]=x[0]/x[2],x[1]=x[1]/x[2];
		mean+=sqrt((images[i].x-x[0])*(images[i].x-x[0])+(images[i].y-x[1])*(images[i].y-x[1]));
	}
	mean/=no_points;
	return mean;
}
double my_projection_error2(vector<Point3f> objects,vector<Point2f> images,Mat K,Mat R,Mat T,int no_points){
	vector<Point2f> new_images;
	Mat dist_coeffs=Mat::zeros(8,1,CV_64F);
	projectPoints(objects,R,T,K,dist_coeffs,new_images);
	double mean=0;
	int i;
	for (i=0;i<no_points;i++){
		mean+=sqrt((images[i].x-new_images[i].x)*(images[i].x-new_images[i].x)+(images[i].y-new_images[i].y)*(images[i].y-new_images[i].y));
	}
	mean/=no_points;
	return mean;
}
int main(int argc,char *argv[]){
	if (argc<2){
		printf("Error: Wrong input format\n");
		return -1;
	}
	FILE *fp=fopen(argv[1],"r");
	int image_no,no_points,i;
	while (1){
		fscanf(fp,"%d%d",&image_no,&no_points);
		if (feof(fp)) break;
		double a,b,x,y,z;
		vector<Point3f> objects;
		vector<Point2f> images;
		for (i=0;i<no_points;i++){
			fscanf(fp,"%lf%lf%lf%lf%lf",&a,&b,&x,&y,&z);
			images.push_back(Point2f(a,b));
			objects.push_back(Point3f(x,y,z));
		}
		Mat P1,P2,P3,K1,K2,K3,K4,R1,R2,R3,R4,T1,T2,T3,T4;
		P1=basic_lse(objects,images,&K1,&R1,&T1,no_points);
		P2=my_svd(objects,images,&K2,&R2,&T2,no_points);
		P3=my_ransac(objects,images,&K3,&R3,&T3,no_points);
		in_built(objects,images,&K4,&R4,&T4,no_points);
/*		cout<<K1<<endl<<endl;
		cout<<K2<<endl<<endl;
		cout<<K3<<endl<<endl;
		cout<<K4<<endl<<endl;*/
		cout<<"The errors for image "<<image_no<<endl;
		cout<<"The mean projection error with only 6 points is "<<my_projection_error1(objects,images,P1,no_points)<<endl;
		cout<<"The mean projection error with all points using SVD is "<<my_projection_error1(objects,images,P2,no_points)<<endl;
		cout<<"The mean projection error using RANSAC method is "<<my_projection_error1(objects,images,P3,no_points)<<endl;
		cout<<"The mean projection error using the bulit initCamera and SolvePnP is "<<my_projection_error2(objects,images,K4,R4,T4,no_points)<<endl;
		cout<<endl;
	}
}
