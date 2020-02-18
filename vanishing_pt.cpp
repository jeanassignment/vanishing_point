#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdint.h>
#include <cmath>



using namespace std;


//****************************************************************************
//  Global Parameters
//****************************************************************************


// canny edge
float canny_lowthreshold = 100; 
float canny_highthreshold = canny_lowthreshold * 3;  //Canny’s recommendation 2-3*lower

// hough transform
int hough_threshold = 150; 		// [number of intersect] minimum number to agree to a line 
int min_line_len = 100;	   		// [px] minimum detected length of line, bigger to avoid noise
int max_line_gap = 10;			// [px] max gap to differentiate lines, higher to get more continuous lines, slower

// remove/merge line criteria
float vertical_deg = 85;   		// [deg] filter out line angle bigger than this
float merge_line_ang = 0.005;   	// [deg] intersect smaller than this angle 
float merge_line_dist = 3; 	    // [px] orthogonal dist smaller than this distance 

// vanishing point criteria
int vicinity = 10; 			    // [px] points lies within this vicinity of a proposed point are same cluster
int min_ratio = 100; 			// [ratio of points] define vanishing points (cluster includes more than certain ratio of points), higher the less points

// drawing colors: cyan, green, red, magenta, blue, yellow
cv::Scalar colors[6] = {cv::Scalar(0,255,0), cv::Scalar(255,255,0), cv::Scalar(0,0,255), cv::Scalar(0,255,255) ,cv::Scalar(255,0,0), cv::Scalar(255,0,255)};
	

//****************************************************************************
//  Classes
//****************************************************************************

class LineSegment{

	private: 
		cv::Point left, right;  // end points of line segment
		cv::Vec3f params;    	// 3 parameters a,b,c of line eq.
		float m; 				//gradient m = -a/b
		
	public:

		LineSegment(cv::Vec4i l){
			
			if(l[1]<l[3]){
				left.x = l[0];
				left.y = l[1];
				right.x = l[2];
				right.y = l[3];
			}else{
				left.x = l[2];
				left.y = l[3];
				right.x = l[0];
				right.y = l[1];
			}
			
			params[0] = (left.y - right.y);
			params[1] = (right.x - left.x);
			params[2] = (left.x * right.y - right.x * left.y);

			m = -params[0] / params[1];

		};
		
		
		cv::Vec3f getlineparam(){
			return params;
		};
		cv::Point lpoint(){
			return left;
		}
		cv::Point rpoint(){
			return right;
		}


		float crossangle(const LineSegment& l2){
			return abs(atan2(l2.m-m, (1+m*l2.m)));
		}

		cv::Point crosspoint(const LineSegment& l2){
			
			cv::Point cross;
			float d = params[0] * l2.params[1] - l2.params[0] * params[1];// determinative a1 * b2 - a2 * b1  
			float angle = crossangle(l2);
			
			if (angle < merge_line_ang/180*CV_PI){  // parallel lines   0.001 (0.5 deg)
				return cv::Point(0,0);
			}
			cross.x = (params[1] * l2.params[2] - l2.params[1] * params[2]) / d; // (b1 * c2 - b2 * c1) / d
			cross.y = (l2.params[0] * params[2] - params[0] * l2.params[2]) / d; // (a2 * c1 - a1 * c2) / d
			return cross;
		}

		void draw(cv::Mat& img, cv::Scalar color){
			line( img, left, right, color, 3, cv::LINE_AA);
		}

		friend bool isEqual(const LineSegment& l1, const LineSegment& l2);
	
		
};

bool isEqual (const LineSegment& l1, const LineSegment& l2){

	// angle difference
	float ang = abs(atan2(l2.m-l1.m, (1+l1.m*l2.m)));
			
	// orthogonal distance  |ax+by+c|/sqrt(a*a+b+b)
	float dist = abs(l1.params[0] * l2.left.x + l1.params[1] * l2.left.y + l1.params[2])/sqrt(l1.params[0]*l1.params[0] + l1.params[1]*l1.params[1]);  
	float dist2 = abs(l1.params[0] * l2.right.x + l1.params[1] * l2.right.y + l1.params[2])/sqrt(l1.params[0]*l1.params[0] + l1.params[1]*l1.params[1]); 
 
	if (max(dist, dist2) < merge_line_dist && ang < (merge_line_ang / 180 * CV_PI)){
		return true;
	}
		return false;
};

class VanishingPoint{
	private:
		cv::Point p;
		double residuals = 0;
		int line1_id, line2_id;  // the line combination
		
	public:
	    // int group_id = -1;  // -1: ungrouped
		VanishingPoint(cv::Point pt):
			p(pt)
		{ 
			
		}

		void set_residuals(vector<LineSegment> lines){
			// calculate accumulated point to line distances
			for (size_t k = 0; k < lines.size(); k++ ){
				cv::Vec3f params = lines[k].getlineparam();
				double a = params[0], b = params[1], c = params[2];
				residuals += abs(a*p.x + b*p.y + c) / sqrt(a*a + b*b);
			}
		}

		void set_lines(int l1, int l2){
			line1_id = l1;
			line2_id = l2;
		}
		int l1(){ return line1_id;}
		int l2(){ return line2_id;}
		

		bool operator < (VanishingPoint& B){
        	if (residuals < B.residuals) {
            	return true;
        	}else return false;
		}

		double operator - (VanishingPoint& p2){
		    double distance = sqrt((p2.p.x - p.x)*(p2.p.x - p.x)  + (p2.p.y - p.y)*(p2.p.y - p.y));
		}
		void draw(cv::Mat img, cv::Scalar color){
			drawMarker(img, p, color, cv::MARKER_CROSS, 100, 10, 16);
		}

};


//****************************************************************************
//  Vanishing Point Algorithm
//****************************************************************************


int main(int argc, char** argv) {

	//-----Image Containers-----
	cv::Mat image, image_grey, image_blurred;  		     // preprocess
	cv::Mat image_Cannymask, image_edges;	   			 // canny edge
	cv::Mat image_detected, image_reduced, image_result; // hough, merged, vanishing point 

	//-----1. Preprocess Image-----
	string filename = argv[1]; 
	image = cv::imread(filename, cv::IMREAD_COLOR);
	if (!image.data) { return 0; }							// check img

	cv::cvtColor(image, image_grey, cv::COLOR_BGR2GRAY);	// grey scale
	cv::blur(image_grey, image_blurred, cv::Size(3, 3));	// blur image
	
	//-----2. Canny Edge Detection-----
	cv::Canny(image_blurred, image_Cannymask, canny_lowthreshold, canny_highthreshold, 3);
	
	image_edges = cv::Scalar::all(0); 						// draw 
	image_grey.copyTo(image_edges, image_Cannymask);		
	cv::cvtColor(image_Cannymask, image_detected, cv::COLOR_GRAY2BGR);
	cv::cvtColor(image_Cannymask, image_reduced, cv::COLOR_GRAY2BGR);
	cv::cvtColor(image_Cannymask, image_result, cv::COLOR_GRAY2BGR);
	cv::namedWindow("Canny", cv::WINDOW_NORMAL);
 	cv::resizeWindow("Canny", 800, 800);
	cv::imshow("Canny", image_result);

	//-----3. Probabilistic_Hough Transform & Post Processing-----
   	
	// lines containers
	vector<cv::Vec4i> p_lines;			 // all segments from HT
	vector<LineSegment> detected_lines;  // non-vertical lines
	vector<LineSegment> merged_lines;	 // merged lines (that are close to each other by distance/angle)
    
	// detect lines
	cv::HoughLinesP( image_edges, p_lines, 1, CV_PI/180, hough_threshold, min_line_len, max_line_gap );
	
	// remove vertical lines
	for( size_t i = 0; i < p_lines.size(); i++ )
   	{
      cv::Vec4i l = p_lines[i];
	  float ang = atan2(abs(p_lines[i][1] - p_lines[i][3]), abs(p_lines[i][0] - p_lines[i][2])) / CV_PI * 180;
		if (ang > vertical_deg){
			line( image_detected, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), colors[2], 5, cv::LINE_AA);
		}else{
			LineSegment line(p_lines[i]);
			line.draw(image_detected, colors[4]);
			detected_lines.push_back(line);
		}
    }
	cv::namedWindow("Detected", cv::WINDOW_NORMAL);		// draw
 	cv::resizeWindow("Detected", 800, 800);
	cv::imshow("Detected", image_detected);
	

	// group lines in dictionary (based on line angle and distance)
	std::vector<int> labels;
	cv::partition(detected_lines, labels, isEqual);
	map<int, vector<int>> lmap;  // [key]: grouplabel, [value]: memebers id in detected_lines
	for (size_t i = 0; i< labels.size(); i++){
		lmap[labels[i]].push_back(i);
	}

	// merge lines (find left-most, right-most points in same group -> new linesegments)
	for (size_t i = 0; i< lmap.size(); i++){  //each group
		
		cv::Point left_most(0, image.cols), right_most(0, 0);
		
		for (size_t j = 0; j < lmap[i].size(); j++){ // each line in group

			LineSegment current = detected_lines[lmap[i][j]];
			if (current.lpoint().y < left_most.y) {
				left_most = current.lpoint();
			}
			if(current.rpoint().y > right_most.y){
				right_most = current.rpoint();
			}
		}
		LineSegment merged_line(cv::Vec4i(left_most.x, left_most.y, right_most.x, right_most.y));
		merged_line.draw(image_reduced, colors[1]);
		merged_lines.push_back(merged_line);
	}
    
	cv::namedWindow("Reduced", cv::WINDOW_NORMAL);		// draw 
 	cv::resizeWindow("Reduced", 800, 800);
	cv::imshow("Reduced", image_reduced);


	
	//-----4. Points Proposal & Sort-----
	// points container
	vector<VanishingPoint> vps;
	
	// each intersection of two lines (i,j)
    for( size_t i = 0; i < merged_lines.size(); i++ )
    {
	  	cv::Point intersection;
	  	for (size_t j = i + 1; j < merged_lines.size(); j++) {
				
			intersection = merged_lines[i].crosspoint(merged_lines[j]);
			
			if (intersection.x > 0 && intersection.x < image.rows && intersection.y > 0 && intersection.y < image.cols){ // intersect within images
				VanishingPoint vp(intersection);
				vp.set_lines(i,j);
				vp.set_residuals(merged_lines);  //(accumulated distance to all lines)
				vps.push_back(vp);
			}
		}
    }
	
	// sort with residuals (i.e. first point = main vanishing point)
	sort(vps.begin(), vps.end());
	
	
	//----- 5. Find Vanishing Point (Clusters)-----
	
	/*  method: 
		a. use first element in sorted "vps" as key point of the first cluster
		b. loop thru all points in "vps", group points within vicinity to this cluster
		c. if cluster is big enough --> vanishing point
		d. use next ungrouped point in "vps" and far from exsiting vanishing points
		e. go to b and repeat, till no nextkey available
	*/

	// init point id(first element in vps)
	int key = 0;	    // proposed key point
	int nextkey = -1;   // next key (-1:not assigned)
	int count = 0;		// numbers of vanishing cluster
	int chosedvp = key;		// chosen vp id
	vector<int> cluster_vps_id;  //store id belong to this cluster
	vector<int> vpointkey;  // all vanishing points(verified as a big cluster)
	

	while(key != -1){ // if the key is not assigned (exhausted thru points)
		
		vpointkey.push_back(chosedvp);
		
		for (size_t j = key+1; j < vps.size(); j++){  
			double dist = vps[j] - vps[key];   //current point dist to current key
			
			if (dist < vicinity){ 
				
				cluster_vps_id.push_back(j);

			}else if (nextkey == -1){ 
				bool pass = 1;
				double dist_to_p = 0;
				for(size_t k = 0; k < vpointkey.size(); k++){ 
					dist_to_p = vps[j] - vps[vpointkey[k]];	// current key to previous vanishing points
					if (dist_to_p < 3*vicinity){			// if too close to any of the previous points, do not propose as next key point 
						pass = 0;
						break;
					}
				}
				if (pass == 1){	// propose as next key point
					nextkey = j;
				}
			}
		}

		if(cluster_vps_id.size() > (vps.size()/min_ratio)){  // when the cluster size is large enough -> vanishing point
			
			vps[key].draw(image_result, colors[count%6]);
			merged_lines[vps[key].l1()].draw(image_result, colors[count%6]);
			merged_lines[vps[key].l2()].draw(image_result, colors[count%6]);

			for (size_t j = 0; j < cluster_vps_id.size(); j++){
			   merged_lines[vps[cluster_vps_id[j]].l1()].draw(image_result, colors[count%6]);
			   merged_lines[vps[cluster_vps_id[j]].l2()].draw(image_result, colors[count%6]);
			}
			chosedvp = key;
			count++;
		}
		
		// update
		
		key = nextkey;
		nextkey = -1;
		cluster_vps_id.clear();
		
	}
	

	//****************************************************************************
	//  Logs
	//****************************************************************************

	cout << endl;
	cout << "Report:"<< endl;
	cout << "-----------Hough Transformation-----------"<< endl;
	cout << "detected lines: " << p_lines.size()<<endl;
	cout << "detected lines (without vertical lines): " << detected_lines.size()<<endl;
	cout << "reduced lines (after merging): " << merged_lines.size()<< endl;
	cout << endl;
	cout << "-----------Vanishing Points-----------"<< endl;
	cout << "proposed points: " << vps.size()<<endl;
	cout << "clusters > 1/" << min_ratio <<" points agrees ( "<< vps.size()/min_ratio <<" points lies in ± "<< vicinity <<" px range of center): " << count <<endl;
	

	cv::namedWindow("Result", cv::WINDOW_NORMAL);
    cv::resizeWindow("Result", 800, 800);
	cv::imshow("Result", image_result);
	cv::imwrite("result/result_"+filename, image_result);

	
	cv::waitKey(0);
	return(0);
}


