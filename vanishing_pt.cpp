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
int min_line_len = 50;	   		// [px] minimum detected length of line, bigger to avoid noise
int max_line_gap = 10;			// [px] max gap to differentiate lines, higher to get more continuous lines, slower

// remove/merge line criteria
float vertical_deg = 85;   		// [deg] filter out line angle bigger than this
float merge_line_ang = 0.5;   	// [deg] intersect smaller than this angle 
float merge_line_dist = 2; 	    // [px] orthogonal dist smaller than this distance 

// voting for points
int vote_dist = 5;  			// rough 

// vanishing point criteria
int vicinity = 5; 			    // [px] points lies within this vicinity of a proposed point are same cluster (should be bigger than merge_line_dist)
int min_ratio = 50; 			// [ratio of points] relate to avg line angles, 300 for 5DK, 50 for 5DL image

// drawing colors: cyan, green, red, magenta, blue, yellow
cv::Scalar colors[6] = {cv::Scalar(0,255,0), cv::Scalar(255,255,0), cv::Scalar(0,0,255), cv::Scalar(0,255,255) ,cv::Scalar(255,0,0), cv::Scalar(255,0,255)};
	

//****************************************************************************
//  Classes
//****************************************************************************

class LineSegment{

	private: 
		cv::Point left, right;  // end points of line segment
		cv::Vec3f params;    	// 3 parameters a,b,c of line eq.
		float m; 				//gradient m = -a/b (slope)

		int vanishing_point = -1;	// agreed vanishing line, -1: unset
		float vanishing_dist = -1;
		
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
		
		float get_angle(){
		
			return abs(atan2(right.y - left.y , abs(right.x - left.x)));
		}
		cv::Vec3f getlineparam(){
			return params;
		};
		cv::Point lpoint(){
			return left;
		}
		cv::Point rpoint(){
			return right;
		}
		int get_vp(){
			return vanishing_point;
		}

		void set_vp(int id, float dist){
			vanishing_dist = dist;
			vanishing_point = id;
		}

		bool vp_set(){
			if (vanishing_point != -1){
				return true;
			}
		}

		float vp_dist(){
			return vanishing_dist;
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
	
	// y coordinate distance 
	float ydist;
	if (l1.left.y > l2.right.y){  //l1 is at right
		ydist = l1.left.y - l2.right.y;
	}else{
		ydist = l2.left.y - l1.right.y;
	}

	if (max(dist, dist2) < merge_line_dist && ang < (merge_line_ang / 180 * CV_PI) && ydist < 2*max_line_gap){
		return true;
	}
		return false;
};

class CrossPoint{
	private:
		cv::Point p;
		int point_id;			 // in cps vector "after sort!"
		int votes = 0;			 // lines roughly agree to the point
		int line1_id, line2_id;  // the line combination
		
		bool vp = 0;		 	// if it's a vanishing point
		vector<int> vp_lid;  	// all the lines strongly agree to this vanishing point
		
	public:
	    // int group_id = -1;  // -1: ungrouped
		CrossPoint(cv::Point pt):
			p(pt)
		{ 
			
		}

		void set_pointid(int id){  //its position aftering sorting
			point_id = id;
		}

		void set_vp(){
			vp = 1;
		}

		bool is_vp(){
			return vp;
		}

		vector<int> get_vp_lid(){
			return vp_lid;
		}
		
		void add_vp_lid(int lineid){
			vp_lid.push_back(lineid);
		}

		void remove_vp_lid(int lineid){
			vector<int>::iterator it = find(vp_lid.begin(), vp_lid.end(), lineid);
			vp_lid.erase(it);
		}

		void lines_vote(vector<LineSegment> lines){
			
			for (size_t k = 0; k < lines.size(); k++ ){
				float dist = this->dist2line(lines[k]);
				if (dist < vote_dist){
					votes++;
				}
			}
		}

		void set_lines(int l1, int l2){
			line1_id = l1;
			line2_id = l2;
		}
		int l1(){ return line1_id;}
		int l2(){ return line2_id;}
		
		float dist2line(LineSegment l){
			cv::Vec3f params = l.getlineparam();
			double a = params[0], b = params[1], c = params[2];
			return abs(a*p.x + b*p.y + c) / sqrt(a*a + b*b);
		}

		bool operator < (CrossPoint& B){
        	if (votes > B.votes) {
            	return true;
        	}else return false;
		}

		double operator - (CrossPoint& p2){
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
	string filename = "5D4L1L1D_L.jpg";   //caution!!  5D4KVN2Y_R: min_ratio set to 300, 5D4L1L1D_L:50 (not good, need to find relation of image line angle to min_ratio)
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
			line.draw(image_detected, colors[1]);
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
	float avg_angle = 0;
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
		avg_angle += merged_line.get_angle() * 180 / CV_PI;
		merged_line.draw(image_reduced, colors[1]);
		merged_lines.push_back(merged_line);
	}
	avg_angle/=merged_lines.size();
	// calculate average line angle
	
    
	cv::namedWindow("Reduced", cv::WINDOW_NORMAL);		// draw 
 	cv::resizeWindow("Reduced", 800, 800);
	cv::imshow("Reduced", image_reduced);


	
	//-----4. Points Proposal & Sort-----
	// points container
	vector<CrossPoint> cps;
	int point_id = 0;
	
	// each intersection of two lines (i,j)
	int ls = merged_lines.size();
    for( size_t i = 0; i < merged_lines.size(); i++ )
    {
	  	cv::Point intersection;
	  	for (size_t j = i + 1; j < merged_lines.size(); j++) {
				
			intersection = merged_lines[i].crosspoint(merged_lines[j]);
			
			if (intersection.x > 0 && intersection.x < image.rows && intersection.y > 0 && intersection.y < image.cols){ // intersect within images
				CrossPoint cp(intersection);
				cp.set_lines(i,j);
				cp.lines_vote(merged_lines);  //(accumulated distance to all lines)
				cps.push_back(cp);
				point_id++;
			}
		}
    }
	
	// sort with residuals (i.e. first point = main vanishing point)
	sort(cps.begin(), cps.end());
	for (size_t i = 0; i < cps.size(); i++){
		cps[i].set_pointid(i);
	}
	
	//----- 5. Find Vanishing Point (Clusters)-----
	
	/*  method: 
		a. use first element in sorted "cps" as key point of the first cluster
		b. loop thru all points in "cps", group points within vicinity to this cluster
		c. if cluster is big enough --> vanishing point
		d. use next ungrouped point in "cps" and far from exsiting vanishing points
		e. go to b and repeat, till no nextkey available
	*/

	// init point id(first element in cps)
	int key = 0;	    	   // proposed key point
	int nextkey = -1;   	   // next key (-1:not assigned)
	vector<int> cluster_pid;   // store point id belong to this cluster, renew with key 
	
	int count = 0;			   	 // numbers of vanishing cluster
	vector<int> vanishing_pid;   // all vanishing points id (big cluster)
	
	while(key != -1){ // if the key is not assigned (exhausted thru points)
		
		
		for (size_t j = key+1; j < cps.size(); j++){  
			double dist = cps[j] - cps[key];   //current point dist to current key
			
			if (dist < vicinity){ 
				cluster_pid.push_back(j);
			}else if (nextkey == -1){ 
				nextkey = j;	
			}
		}
		int s = cluster_pid.size();
		if(cluster_pid.size() > cps.size()/min_ratio){  // when the cluster size is large enough -> vanishing point
			cps[key].set_vp();
			vanishing_pid.push_back(key);

			// all points in cluster
			for (size_t i = 0; i < cluster_pid.size(); i++){
				// two lines
				int l1 = cps[cluster_pid[i]].l1();
				int l2 = cps[cluster_pid[i]].l2();
				
				// key point distance to these lines
				float dist1 = cps[key].dist2line(merged_lines[l1]);
				float dist2 = cps[key].dist2line(merged_lines[l2]);

				// update line1
				if (!merged_lines[l1].vp_set()){	// if the line not assigned to any vp yet
					merged_lines[l1].set_vp(key, dist1);
					cps[key].add_vp_lid(l1);
				}else if(dist1 < merged_lines[l1].vp_dist()){
					cps[merged_lines[l1].get_vp()].remove_vp_lid(l1); // remove from the previous agreed vp
					cps[key].add_vp_lid(l1);
					merged_lines[l1].set_vp(key, dist1);
				}
				// update line2
				if (!merged_lines[l2].vp_set()){	// if the line not assigned to any vp yet
					merged_lines[l2].set_vp(key, dist2);
					cps[key].add_vp_lid(l2);
				}else if(dist1 < merged_lines[l2].vp_dist()){
					cps[merged_lines[l2].get_vp()].remove_vp_lid(l2); // remove from the previous agreed vp
					cps[key].add_vp_lid(l2);
					merged_lines[l2].set_vp(key, dist2);
				}

			}

			

			count++;
		}

			
		// update
		
		key = nextkey;
		nextkey = -1;
		cluster_pid.clear();
		
	}
	
	// draw for all vpoint
		for (size_t i = 0; i < vanishing_pid.size(); i++){
			int pid =vanishing_pid[i];
			vector<int> lid = cps[pid].get_vp_lid();
			
			// itself and its line
			cps[pid].draw(image_result, colors[i%6]);
			merged_lines[cps[pid].l1()].draw(image_result, colors[i%6]);
			merged_lines[cps[pid].l2()].draw(image_result, colors[i%6]);

			// other lines agree to it
			for (size_t j = 0; j < lid.size(); j++){
				merged_lines[lid[j]].draw(image_result, colors[i%6]);
			}
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
	cout << "proposed points: " << cps.size()<<endl;
	cout << "clusters > 1/" << min_ratio <<" points agrees ( "<< cps.size()/min_ratio <<" points lies in ± "<< vicinity <<" px range of center): " << count <<endl;
	

	cv::namedWindow("Result", cv::WINDOW_NORMAL);
    cv::resizeWindow("Result", 800, 800);
	cv::imshow("Result", image_result);
	cv::imwrite("result/result_"+filename, image_result);

	
	cv::waitKey(0);
	return(0);
}


