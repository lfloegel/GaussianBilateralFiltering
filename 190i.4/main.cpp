#include "pfm.h"
#include "Vector.hpp"

#include <iostream>
#include <math.h>
#include <cstring>

using namespace std;

// indexing 1d image array
int imIdx(int i, int j, int width, int channels = 3) {
	return channels * width * i + channels * j;
}

float* GaussianFilter(float* image, unsigned width, unsigned height) {
	// C_output
	float* gauss = new float[width*height*3];
	memcpy(gauss, image, sizeof(float)*width*height*3);

	// Parameters
	float sigma_p = 5;

	// TODO 1
	// Implement Gaussian Filtering
	//go through each index, calculate the gauss at point
	Vector3f sum_of_weighted_values;
	float sum_of_weights, w_ij;
	for(int w = 0; w < width; w++) //each pixel
	{
		for(int h = 0; h < height; h++)
		{
			sum_of_weighted_values = sum_of_weights = 0.0f;
			for(int jx = w - (2 * sigma_p); jx <= w + (2 * sigma_p); jx++) //21x21 kernel
			{
				for(int jy = h - (2 * sigma_p); jy <= h + (2 * sigma_p); jy++)
				{
					if(jx < 0 || jx >= width || jy < 0 || jy >= height)
					{
						w_ij = 0.0;
					} else {
						float eucl = (float) sqrt(pow(jx - w, 2) + pow(jy - h, 2));
						auto distance = sqrt(eucl);
						w_ij = exp(-1.0 * (powf(distance, 2)) / (2 * (sigma_p * sigma_p)));
						sum_of_weighted_values += w_ij * Vector3f(image[imIdx(jy, jx, width) + 0],
																	image[imIdx(jy, jx, width) + 1],
																	image[imIdx(jy, jx, width) + 2]);
					}
					sum_of_weights += w_ij;
				}
			}
			Vector3f output_img = sum_of_weighted_values / sum_of_weights;
			gauss[imIdx(h, w, width) + 0] = output_img.x; //red
			gauss[imIdx(h, w, width) + 1] = output_img.y; //green
			gauss[imIdx(h, w, width) + 2] = output_img.z; //blue
		}
	}
	
	cout << "Gaussian Filtering -- DONE\n";
	return gauss;
}

float* BilateralFilter(float* image, unsigned width, unsigned height) {
	// C_output
	float* bilateral = new float[width*height*3];
	memcpy(bilateral, image, sizeof(float)*width*height*3);

	// Parameters
	float sigma_p = 5;
	float sigma_c = 0.1;

	// TODO 2
	// Implement Bilateral Filtering
	Vector3f sum_of_weighted_values;
	float sum_of_weights, w_ij;
	for(int w = 0; w < width; w++) //each pixel
	{
		for(int h = 0; h < height; h++)
		{
			sum_of_weighted_values = sum_of_weights = 0.0f;
			for(int jx = w - (2 * sigma_p); jx <= w + (2 * sigma_p); jx++) //21x21 kernel
			{
				for(int jy = h - (2 * sigma_p); jy <= h + (2 * sigma_p); jy++)
				{
					if(jx < 0 || jx >= width || jy < 0 || jy >= height)
					{
						w_ij = 0.0;
					} else {
						float eucl = (float) sqrt(sqrt(pow(jx - w, 2) + pow(jy - h, 2)));
						Vector3f i_colors(image[imIdx(h, w, width) + 0],
										image[imIdx(h, w, width) + 1],
										image[imIdx(h, w, width) + 2]);
						Vector3f j_colors(image[imIdx(jy, jx, width) + 0],
										image[imIdx(jy, jx, width) + 1],
										image[imIdx(jy, jx, width) + 2]);
						float color_eucl = sqrt(powf(i_colors.x - j_colors.x, 2) +
											powf(i_colors.y - j_colors.y, 2)
											+ powf(i_colors.z - j_colors.z, 2));
						w_ij = exp((-1.0 * (powf(eucl, 2)) / (2 * (sigma_p * sigma_p))) -
								(color_eucl * color_eucl / (2 * sigma_c * sigma_c)));
						sum_of_weighted_values += w_ij * Vector3f(image[imIdx(jy, jx, width) + 0],
																	image[imIdx(jy, jx, width) + 1],
																	image[imIdx(jy, jx, width) + 2]);
					}
					sum_of_weights += w_ij;
				}
			}
			Vector3f output_img = sum_of_weighted_values / sum_of_weights;
			bilateral[imIdx(h, w, width) + 0] = output_img.x; //red
			bilateral[imIdx(h, w, width) + 1] = output_img.y; //green
			bilateral[imIdx(h, w, width) + 2] = output_img.z; //blue
		}
	}
	
	
	cout << "Bilateral Filtering -- DONE\n";
	return bilateral;
}

float* JointBilateralFilter(float* image, float* normal, float* position, unsigned width, unsigned height) {
	// C_output
	float* joint = new float[width*height*3];
	memcpy(joint, image, sizeof(float)*width*height*3);

	// Parameters
	float sigma_p = 5;
	float sigma_c = 0.3;
	float sigma_n = 0.1;
	float sigma_d = 0.1;

	// TODO 3
	// Implement Joint Bilateral Filtering
	Vector3f sum_of_weighted_values;
	float sum_of_weights, w_ij;
	for(int w = 0; w < width; w++) //each pixel
	{
		for(int h = 0; h < height; h++)
		{
			sum_of_weighted_values = sum_of_weights = 0.0f;
			for(int jx = w - (2 * sigma_p); jx <= w + (2 * sigma_p); jx++) //21x21 kernel
			{
				for(int jy = h - (2 * sigma_p); jy <= h + (2 * sigma_p); jy++)
				{
					if(jx < 0 || jx >= width || jy < 0 || jy >= height)
					{
						w_ij = 0.0;
					} else {
						float eucl = sqrt(sqrt(pow(jx - w, 2) + pow(jy - h, 2)));
						Vector3f i_colors(image[imIdx(h, w, width) + 0],
										image[imIdx(h, w, width) + 1],
										image[imIdx(h, w, width) + 2]);
						Vector3f j_colors(image[imIdx(jy, jx, width) + 0],
										image[imIdx(jy, jx, width) + 1],
										image[imIdx(jy, jx, width) + 2]);
						float color_eucl = sqrt(powf(i_colors.x - j_colors.x, 2) +
											powf(i_colors.y - j_colors.y, 2)
											+ powf(i_colors.z - j_colors.z, 2));
						/**
						 * joint bilateral
						**/
						//normals
						Vector3f i_normal(normal[imIdx(h, w, width) + 0],
										normal[imIdx(h, w, width) + 1],
										normal[imIdx(h, w, width) + 2]);
						Vector3f j_normal(normal[imIdx(jy, jx, width) + 0],
										normal[imIdx(jy, jx, width) + 1],
										normal[imIdx(jy, jx, width) + 2]);
						float d_normal, dot_p = dotProduct(i_normal, j_normal);
						if(!(dot_p >= -1 && dot_p <= 1))
						{
							d_normal = 0.0;
						} else {
							d_normal = acos(dot_p);
						}

						//positions
						Vector3f i_pos(position[imIdx(h, w, width) + 0],
										position[imIdx(h, w, width) + 1],
										position[imIdx(h, w, width) + 2]);
						Vector3f j_pos(position[imIdx(jy, jx, width) + 0],
										position[imIdx(jy, jx, width) + 1],
										position[imIdx(jy, jx, width) + 2]);
						float position_eucl = powf(i_pos.x - j_pos.x, 2) +
												powf(i_pos.y - j_pos.y, 2)
												+ powf(i_pos.z - j_pos.z, 2);
						float d_plane = 0.0;
						if(position_eucl != 0.0) {
							d_plane = dotProduct(i_normal, (j_pos - i_pos) / position_eucl);
						}
						w_ij = exp((-1.0 * (powf(eucl, 2)) / (2 * (sigma_p * sigma_p))) -
								(color_eucl * color_eucl / (2 * sigma_c * sigma_c)) -
								(d_normal * d_normal / (2 * sigma_n * sigma_n)) -
								(d_plane * d_plane / (2 * sigma_d * sigma_d)));
						sum_of_weighted_values += w_ij * Vector3f(image[imIdx(jy, jx, width) + 0],
																	image[imIdx(jy, jx, width) + 1],
																	image[imIdx(jy, jx, width) + 2]);
					}
					sum_of_weights += w_ij;
				}
			}
			Vector3f output_img = sum_of_weighted_values / sum_of_weights;
			joint[imIdx(h, w, width) + 0] = output_img.x; //red
			joint[imIdx(h, w, width) + 1] = output_img.y; //green
			joint[imIdx(h, w, width) + 2] = output_img.z; //blue
		}
	}
	

	cout << "Joint Bilateral Filtering -- DONE\n";
	return joint;
}

int main() {
    unsigned *w = new unsigned;
    unsigned *h = new unsigned;

	// Input buffers
	float* imageBuffer = read_pfm_file3("im/s2.pfm", w, h);      		 // load image buffer - 3 channels
    float* normalBuffer = read_pfm_file3("im/s2_normal.pfm", w, h);      // load normal buffer - 3 channels
	float* positionBuffer = read_pfm_file3("im/s2_position.pfm", w, h);  // load position buffer - 3 channels

	float* gausssianFiltered = GaussianFilter(imageBuffer, *w, *h);
	write_pfm_file3("im/s2_gaussian.pfm", gausssianFiltered, *w, *h);
	delete[] gausssianFiltered;
	
	float* bilateralFiltered = BilateralFilter(imageBuffer, *w, *h);
	write_pfm_file3("im/s2_bilateral.pfm", bilateralFiltered, *w, *h);
	delete[] bilateralFiltered;
	
	float* jointBilateralFiltered = JointBilateralFilter(imageBuffer, normalBuffer, positionBuffer, *w, *h);
	write_pfm_file3("im/s2_jointBilateral.pfm", jointBilateralFiltered, *w, *h);
	delete[] jointBilateralFiltered;


	// 
	delete[] imageBuffer;
	delete[] normalBuffer;
	delete[] positionBuffer;
	
	return 0;
}