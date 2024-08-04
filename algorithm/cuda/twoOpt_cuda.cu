//
// Created by franklyn on 7/25/24.
//

#include "twoOpt_cuda.cuh"
#include "fitnessComputer_cuda.cuh"
#include <iostream>

__device__ void computeFitness_cuda_single(
                    Paths_cuda paths,
                    int smoothedPathLength,
                     const float* heightMap,
                     int heightMapWidth,
                     float N_wp, int smooth_startIndex, float max_asc_angle,
                     float max_desc_angle, float a_utopia, float f_utopia, float resolution, float &fitness, int index) {

    float w1 = 0.30;
    float w2 = 0.70;

    float d_ug = 0.0;
    float d_dz = 0.0;
    float d_ea = 0.0;
    float l_traj = 0.0;

    float a_cum = 0;
    float a_avg = 0;
    float f_avg = 0;

    int n = smoothedPathLength;

    int totalSteps = 1;

    int underground = 0;
    bool undergroundLast = false;

    float4 P1_v;
    float4 P2_v;

    float steps_P1P2;
    float inv_steps;
    float step_length_P1P2;

    float4 diff;
    float4 interval;
    int2 pointXY;
    float pointZ;
    float currentAltitude;



    for (int i = 0; i < n - 1; i++) {
        P1_v.x = paths.tempSmoothedPaths.elements[smooth_startIndex + i * 3 + 0];
        P1_v.y = paths.tempSmoothedPaths.elements[smooth_startIndex + i * 3 + 1];
        P1_v.z = paths.tempSmoothedPaths.elements[smooth_startIndex + i * 3 + 2];
        P2_v.x = paths.tempSmoothedPaths.elements[smooth_startIndex + (i + 1) * 3 + 0];
        P2_v.y = paths.tempSmoothedPaths.elements[smooth_startIndex + (i + 1) * 3 + 1];
        P2_v.z = paths.tempSmoothedPaths.elements[smooth_startIndex + (i + 1) * 3 + 2];

        diff.x = P2_v.x - P1_v.x;
        diff.y = P2_v.y - P1_v.y;
        diff.z = P2_v.z - P1_v.z;

        float distance_P1P2 = norm3df(
                diff.x,
                diff.y,
                diff.z
        );

        steps_P1P2 = floor(distance_P1P2 * resolution);


        if (steps_P1P2 > 0) {
            inv_steps = 1 / steps_P1P2;
            step_length_P1P2 = distance_P1P2 * inv_steps;
            interval.x = diff.x * inv_steps;
            interval.y = diff.y * inv_steps;
            interval.z = diff.z * inv_steps;
        } else {
            step_length_P1P2 = distance_P1P2;
            interval.x = 0;
            interval.y = 0;
            interval.z = 0;
        }

        float horizontal_length = hypotf(diff.x, diff.y);
        float angle_radians = atan2(diff.z, horizontal_length);


        l_traj += distance_P1P2;
        totalSteps += max(0.0f, steps_P1P2 - 2);
        for (float j = 1.0f; j < steps_P1P2; j++) {
            pointXY.x = __float2int_rn(P1_v.x + interval.x * j);
            pointXY.y = __float2int_rn(P1_v.y + interval.y * j);
            pointZ = P1_v.z + interval.z * j;

            currentAltitude = pointZ - heightMap[pointXY.y * heightMapWidth + pointXY.x];
            underground = currentAltitude < a_utopia;

            d_ug += step_length_P1P2 * underground;
            a_cum += currentAltitude;
        }

        pointXY.x = __float2int_rn(P2_v.x);
        pointXY.y = __float2int_rn(P2_v.y);

        currentAltitude = P2_v.z - heightMap[pointXY.y * heightMapWidth + pointXY.x];

        underground = currentAltitude < a_utopia;
        d_ug += step_length_P1P2 * underground;
        a_cum += currentAltitude;

        if (angle_radians > max_asc_angle || angle_radians < max_desc_angle) {
            d_ea += distance_P1P2;
        }

        totalSteps++;
    }


    //Penaly term P
    float P = d_ug + d_dz + d_ea + (N_wp * l_traj);

    //Fitness function F
    if (P == 0.0) {
        //Cost term C
        a_avg = a_cum / totalSteps;
        float C = w1 * (a_avg / a_utopia) + w2 * (l_traj / f_utopia);

       fitness = 1 + 1 / (1 + C);
    } else {
        fitness = 0 + 1 / (1 + P);
    }

}

__device__ void smoothPath_cuda_single(
        Paths_cuda paths,
        int &smoothedPathLength_r,
        float &N_wps,
		int smooth_startIndex,
        int raw_startIndex,
        float turnRadius, int n_pi) {

    float n_pi_f = __int2float_rn(n_pi);
    float prevCPath = 0.0f;

    float unsmoothedVertices = 0;

	paths.tempSmoothedPaths.elements[smooth_startIndex] = paths.tempPaths.elements[raw_startIndex];
    paths.tempSmoothedPaths.elements[smooth_startIndex + 1] = paths.tempPaths.elements[raw_startIndex + 1];
    paths.tempSmoothedPaths.elements[smooth_startIndex + 2] = paths.tempPaths.elements[raw_startIndex + 2];

    int smoothedPathLength = 1;

    int n = paths.rawPaths.n_waypoints;

    float P[3];
    float P1[3];
    float P2[3];

    float C[3];

    float4 tau_1;
    float4 tau_2;

    float mag_1;
    float mag_2;

    float mag_1_inv;
    float mag_2_inv;

    for (int i = 1; i < n - 1; i++) {
       P1[0] = paths.tempPaths.elements[raw_startIndex + 3 * (i - 1)];
        P1[1] = paths.tempPaths.elements[raw_startIndex + 3 * (i - 1) + 1];
        P1[2] = paths.tempPaths.elements[raw_startIndex + 3 * (i - 1) + 2];

        P[0] = paths.tempPaths.elements[raw_startIndex + 3 * i];
        P[1] = paths.tempPaths.elements[raw_startIndex + 3 * i + 1];
        P[2] = paths.tempPaths.elements[raw_startIndex + 3 * i + 2];

        P2[0] = paths.tempPaths.elements[raw_startIndex + 3 * (i + 1)];
        P2[1] = paths.tempPaths.elements[raw_startIndex + 3 * (i + 1) + 1];
        P2[2] = paths.tempPaths.elements[raw_startIndex + 3 * (i + 1) + 2];

        //unit vector from P1 to P
        mag_1 = sqrt(
                (P[0] - P1[0]) * (P[0] - P1[0]) +
                (P[1] - P1[1]) * (P[1] - P1[1]) +
                (P[2] - P1[2]) * (P[2] - P1[2])
        );

        mag_1_inv = 1 / mag_1;

        tau_1.x = (P[0] - P1[0]) * mag_1_inv;
        tau_1.y = (P[1] - P1[1]) * mag_1_inv;
        tau_1.z = (P[2] - P1[2]) * mag_1_inv;

        //unit vector from P to P2
        mag_2 = sqrt(
                (P2[0] - P[0]) * (P2[0] - P[0]) +
                (P2[1] - P[1]) * (P2[1] - P[1]) +
                (P2[2] - P[2]) * (P2[2] - P[2])
        );

        mag_2_inv = 1 / mag_2;

        tau_2.x = (P2[0] - P[0]) * mag_2_inv;
        tau_2.y = (P2[1] - P[1]) * mag_2_inv;
        tau_2.z = (P2[2] - P[2]) * mag_2_inv;

        float dot = tau_1.x * tau_2.x + tau_1.y * tau_2.y + tau_1.z * tau_2.z;

        //angle alpha between the two unit vectors
        float alpha = M_PI - acos(dot);
        float cscAlpha = 1 / sin(alpha);
        float cscAlphaTurnradius = cscAlpha * turnRadius;

        //compute Center C of tangent circle using C = P + turnRadius * csc(alpha) * (tau_2 - tau_1)
        C[0] = P[0] + cscAlphaTurnradius * (tau_2.x - tau_1.x);
        C[1] = P[1] + cscAlphaTurnradius * (tau_2.y - tau_1.y);
        C[2] = P[2] + cscAlphaTurnradius * (tau_2.z - tau_1.z);

        tau_1.x *= cscAlphaTurnradius;
        tau_1.y *= cscAlphaTurnradius;
        tau_1.z *= cscAlphaTurnradius;
        tau_2.x *= cscAlphaTurnradius;
        tau_2.y *= cscAlphaTurnradius;
        tau_2.z *= cscAlphaTurnradius;

        //calculate distance between P and C
        float distance_PC = sqrt(
                (P[0] - C[0]) * (P[0] - C[0]) +
                (P[1] - C[1]) * (P[1] - C[1]) +
                (P[2] - C[2]) * (P[2] - C[2])
        );

        float c_path = distance_PC * cos(alpha * 0.5f);

        if (distance_PC > min(mag_1, mag_2) ||
            c_path + prevCPath > mag_1 ||
            mag_1 == 0.0 ||
            mag_2 == 0.0 ||
            alpha != alpha) {
            //cannot smooth trajectory

            paths.tempSmoothedPaths.elements[smooth_startIndex + smoothedPathLength * 3 + 0] = P[0];
            paths.tempSmoothedPaths.elements[smooth_startIndex + smoothedPathLength * 3 + 1] = P[1];
            paths.tempSmoothedPaths.elements[smooth_startIndex + smoothedPathLength * 3 + 2] = P[2];
            smoothedPathLength++;

            unsmoothedVertices++;
            prevCPath = c_path;
            continue;
        }

        //compute the number n of waypoints required to draw a circular arc using n = max(3, ceil(n_pi * (pi - alpha)/pi))
        float n_waypoints = max(3.0f, n_pi_f * (M_PI - alpha) / M_PI) - 1.0f;
        float oneOverNWaypoints = (M_PI - alpha) / n_waypoints;

        for (float j = 0; j < n_waypoints; j++) {
            float omega = j * oneOverNWaypoints;
            float cosOmega = cos(omega);
            float cosAlphaOmega = cos(alpha + omega);

            int index = smooth_startIndex + smoothedPathLength * 3;

            paths.tempSmoothedPaths.elements[index + 0] = C[0] - tau_1.x * cosAlphaOmega - tau_2.x * cosOmega;
            paths.tempSmoothedPaths.elements[index + 1] = C[1] - tau_1.y * cosAlphaOmega - tau_2.y * cosOmega;
            paths.tempSmoothedPaths.elements[index + 2] = C[2] - tau_1.z * cosAlphaOmega - tau_2.z * cosOmega;

            smoothedPathLength++;

        }

        prevCPath = c_path;

    }

     paths.tempSmoothedPaths.elements[smooth_startIndex + smoothedPathLength * 3 + 0] = paths.tempPaths.elements[
            raw_startIndex + paths.rawPaths.n_waypoints * 3 - 3];
    paths.tempSmoothedPaths.elements[smooth_startIndex + smoothedPathLength * 3 + 1] = paths.tempPaths.elements[
            raw_startIndex + paths.rawPaths.n_waypoints * 3 - 2];
    paths.tempSmoothedPaths.elements[smooth_startIndex + smoothedPathLength * 3 + 2] = paths.tempPaths.elements[
            raw_startIndex + paths.rawPaths.n_waypoints * 3 - 1];
    smoothedPathLength++;

    smoothedPathLength_r = smoothedPathLength;
    N_wps = unsmoothedVertices / n;
}

__global__ void twoOptCuda(
        Paths_cuda paths, Config config, Drone drone, float a_utopia, float f_utopia, int max_elements) {

    int n_points = paths.rawPaths.n_waypoints;
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    int path_index = paths.twoOptFinishedSolutions[blockIdx.x];

    __shared__ int is[32];
    __shared__ int js[32];

	is[threadIdx.x] = 0;
    js[threadIdx.x] = 0;

    if(threadIdx.x == 0) {
        int index = 0;

        for (int i = 1; i < n_points - 2; i++) {
            for(int j = i + 1; j < n_points - 1; j++) {
                is[index] = i;
                js[index] = j;
                index++;

                if(index >= 32) {
                    paths.twoOptCurrentI[path_index] = i;
                    paths.twoOptCurrentJ[path_index] = j;
                    break;
                }

            }
            if(index >= 32) {
                break;
            }
        }
        //paths.twoOptFinishedSolutions[blockIdx.x] = -1;

    }

    float temp_fitness = 0.0f;
    int smootedPathLength;
    float N_wp;

	int startIndex = path_index * 32 * paths.rawPaths.n_waypoints * 3 + threadIdx.x * paths.rawPaths.n_waypoints * 3;
    int smoothedIndex = path_index * 32 * max_elements * 3 + threadIdx.x * max_elements * 3;

   for(int i = 0; i < paths.rawPaths.n_waypoints * 3; i++) {
      paths.tempPaths.elements[startIndex + i]  = paths.rawPaths.elements[path_index * paths.rawPaths.n_waypoints * 3 + i];
   }

	paths.tempPaths.elements[startIndex + 3 * is[threadIdx.x] + 0] = paths.rawPaths.elements[path_index * paths.rawPaths.n_waypoints * 3 + 3 * js[threadIdx.x] + 0];
    paths.tempPaths.elements[startIndex + 3 * is[threadIdx.x] + 1] = paths.rawPaths.elements[path_index * paths.rawPaths.n_waypoints * 3 + 3 * js[threadIdx.x] + 1];
    paths.tempPaths.elements[startIndex + 3 * is[threadIdx.x] + 2] = paths.rawPaths.elements[path_index * paths.rawPaths.n_waypoints * 3 + 3 * js[threadIdx.x] + 2];

    paths.tempPaths.elements[startIndex + 3 * js[threadIdx.x] + 0] = paths.rawPaths.elements[path_index * paths.rawPaths.n_waypoints * 3 + 3 * is[threadIdx.x] + 0];
    paths.tempPaths.elements[startIndex + 3 * js[threadIdx.x] + 1] = paths.rawPaths.elements[path_index * paths.rawPaths.n_waypoints * 3 + 3 * is[threadIdx.x] + 1];
    paths.tempPaths.elements[startIndex + 3 * js[threadIdx.x] + 2] = paths.rawPaths.elements[path_index * paths.rawPaths.n_waypoints * 3 + 3 * is[threadIdx.x] + 2];

    smoothPath_cuda_single(paths, smootedPathLength, N_wp, smoothedIndex, startIndex, drone.turn_radius, 10);

    computeFitness_cuda_single(paths, smootedPathLength, config.heightMap_cuda, config.heightMap_rows, N_wp, smoothedIndex, drone.max_asc_angle, drone.max_desc_angle, a_utopia, f_utopia, config.resolution, temp_fitness, threadIdx.x);


	float f = temp_fitness;

	for (int i = 1; i < 32; i *= 2) {
        f = max(f, __shfl_xor_sync(0xffffffff, f, i));
    }


	if(f > paths.fitnesses[path_index]) {
		//if we are the thread with the best path and its better than unchanged, make the switch permanent.
		//then start again with i, j = 0
		if (f == temp_fitness) {

			paths.rawPaths.elements[path_index * paths.rawPaths.n_waypoints * 3 + 3 * is[threadIdx.x] + 0] = paths.tempPaths.elements[startIndex + 3 * is[threadIdx.x] + 0];
			paths.rawPaths.elements[path_index * paths.rawPaths.n_waypoints * 3 + 3 * is[threadIdx.x] + 1] = paths.tempPaths.elements[startIndex + 3 * is[threadIdx.x] + 1];
			paths.rawPaths.elements[path_index * paths.rawPaths.n_waypoints * 3 + 3 * is[threadIdx.x] + 2] = paths.tempPaths.elements[startIndex + 3 * is[threadIdx.x] + 2];

			paths.rawPaths.elements[path_index * paths.rawPaths.n_waypoints * 3 + 3 * js[threadIdx.x] + 0] = paths.tempPaths.elements[startIndex + 3 * js[threadIdx.x] + 0];
			paths.rawPaths.elements[path_index * paths.rawPaths.n_waypoints * 3 + 3 * js[threadIdx.x] + 1] = paths.tempPaths.elements[startIndex + 3 * js[threadIdx.x] + 1];
			paths.rawPaths.elements[path_index * paths.rawPaths.n_waypoints * 3 + 3 * js[threadIdx.x] + 2] = paths.tempPaths.elements[startIndex + 3 * js[threadIdx.x] + 2];

		    paths.fitnesses[path_index] = f;

		    paths.twoOptCurrentI[path_index] = 0;
		    paths.twoOptCurrentJ[path_index] = 0;

			//printf("%f ", f);
    	}
	} else {
	    //paths.twoOptFinishedSolutions[blockIdx.x] = -1;
	}

  //  printf("%i %i %i %i %i %f %i %f %f %f %f\n", path_index, threadIdx.x, n_points, is[threadIdx.x], js[threadIdx.x], N_wp, smootedPathLength, temp_fitness, f, priorfitness, paths.fitnesses[path_index]);


}