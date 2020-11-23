#include "me_estimator.h"
#include "metric.h"
#include <string>
#include <iostream>


MotionEstimator::MotionEstimator(size_t width, size_t height, unsigned char quality, bool use_half_pixel)
    : width(width)
    , height(height)
    , quality(quality)
    , use_half_pixel(use_half_pixel)
    , width_ext(width + 2 * BORDER)
    , num_blocks_hor((width + BLOCK_SIZE - 1) / BLOCK_SIZE)
    , num_blocks_vert((height + BLOCK_SIZE - 1) / BLOCK_SIZE)
    , first_row_offset(width_ext * BORDER + BORDER)
    , me_field(num_blocks_hor, num_blocks_vert, BLOCK_SIZE)
    , width_borders(width + 2 * BORDER)
    , height_borders(height + 2 * BORDER)
{
    cur_Y_borders = new unsigned char[width_borders * height_borders];
    prev_Y_borders = new unsigned char[width_borders * height_borders];
    if (use_half_pixel) {
        prev_Y_up_borders = new unsigned char[width_borders * height_borders];
        prev_Y_up_left_borders = new unsigned char[width_borders * height_borders];
        prev_Y_left_borders = new unsigned char[width_borders * height_borders];
    }
    // PUT YOUR CODE HERE
}

MotionEstimator::~MotionEstimator() {
    delete[] cur_Y_borders;
    delete[] prev_Y_borders;
    if (use_half_pixel) {
        delete[] prev_Y_up_borders;
        delete[] prev_Y_up_left_borders;
        delete[] prev_Y_left_borders;
    }
    // PUT YOUR CODE HERE
}

void SortTops(uint8_t* tops, const uint32_t* probabilities)
{
    int i, third;
    for (i = 0; i < 8; ++i)
    {
        tops[i] = i;
    }
    for (i = 0; i < 8; ++i)
    {
        for (int j = i+1; j < 8; ++j)
        {
            if (probabilities[tops[i]] < probabilities[tops[j]])
            {
                third = tops[i];
                tops[i] = tops[j];
                tops[j] = third;
            }
        }
    }
}

void MotionEstimator::CEstimate(const unsigned char* cur_Y,
                               const unsigned char* prev_Y,
                               const uint8_t* prev_Y_up,
                               const uint8_t* prev_Y_left,
                               const uint8_t* prev_Y_upleft,
                               uint32_t* probabilities,
                               MEField& mvectors) {
    std::unordered_map<ShiftDir, const uint8_t*, ShiftDirHash> prev_map {
        { ShiftDir::NONE, prev_Y }
    };

    if (use_half_pixel) {
        prev_map.emplace(ShiftDir::UP, prev_Y_up);
        prev_map.emplace(ShiftDir::LEFT, prev_Y_left);
        prev_map.emplace(ShiftDir::UPLEFT, prev_Y_upleft);
    }

    for (size_t i = 0; i < num_blocks_vert; ++i)
    {
        for (size_t j = 0; j < num_blocks_hor; ++j)
        {
            const auto hor_offset = j * BLOCK_SIZE;
            const auto vert_offset = first_row_offset + i * BLOCK_SIZE * width_ext;
            const auto cur = cur_Y + vert_offset + hor_offset;

            MV best_vector;
            best_vector.error = std::numeric_limits<long>::max();

            // PUT YOUR CODE HERE
            
            int points_checked = 8;
            uint8_t ordered_tops[points_checked];
            //SortTops(ordered_tops, &probabilities[i*num_blocks_hor*points_checked+j*points_checked]);
            //std::cout << i << " " << j << " " << i*num_blocks_hor*points_checked+j*points_checked << std::endl;


            for (const auto& prev_pair : prev_map)
            {
                const auto prev = prev_pair.second + vert_offset + hor_offset;

                int previous_step = 0;
                std::pair<int, int> checked_coords[points_checked];
                checked_coords[0] = std::make_pair(STEP, 0); // up
                checked_coords[1] = std::make_pair(STEP, STEP);
                checked_coords[2] = std::make_pair(0, STEP); // right
                checked_coords[3] = std::make_pair(0-STEP, STEP);
                checked_coords[4] = std::make_pair(0-STEP, 0); // down
                checked_coords[5] = std::make_pair(0-STEP, 0-STEP);
                checked_coords[6] = std::make_pair(0, 0-STEP); // left
                checked_coords[7] = std::make_pair(STEP, 0-STEP);

                int first_step;
                for (first_step = 0; first_step < points_checked; ++first_step)
                {
                    const auto comp = prev + checked_coords[first_step].first * width_ext + checked_coords[first_step].second;
                    const int error = GetErrorSAD_16x16(cur, comp, width_ext);
                    //std::cout << "16x16 " << first_step << " " << error << std::endl;
                    if (error < best_vector.error)
                    {
                        best_vector.x = checked_coords[first_step].second;
                        best_vector.y = checked_coords[first_step].first;
                        best_vector.shift_dir = prev_pair.first;
                        best_vector.error = error;
                        previous_step = first_step;
                    }
                    /*
                    if (error < PROB_ERROR_16)
                    {
                        //std::cout << "16x16 " << first_step << " " << error << std::endl;
                        break;
                    }
                    */
                }
                //++probabilities[i*num_blocks_hor*points_checked+j*points_checked+ordered_tops[first_step]];

                for (int step = int(STEP/2); step > 0; step=int(step/2))
                {
                    int prev_x = checked_coords[previous_step].second;
                    int prev_y = checked_coords[previous_step].first;
                    checked_coords[0] = std::make_pair(prev_y+step, prev_x); // up
                    checked_coords[1] = std::make_pair(prev_y+step, prev_x+step);
                    checked_coords[2] = std::make_pair(prev_y, prev_x+step); // right
                    checked_coords[3] = std::make_pair(prev_y-step, prev_x+step);
                    checked_coords[4] = std::make_pair(prev_y-step, prev_x); // down
                    checked_coords[5] = std::make_pair(prev_y-step, prev_x-step);
                    checked_coords[6] = std::make_pair(prev_y, prev_x-step); // left
                    checked_coords[7] = std::make_pair(prev_y+step, prev_x-step);
                    for (int k = 0; k < points_checked; ++k)
                    {
                        const auto comp = prev + checked_coords[k].first * width_ext + checked_coords[k].second;
                        const int error = GetErrorSAD_16x16(cur, comp, width_ext);
                        if (error < best_vector.error)
                        {
                            best_vector.x = checked_coords[k].second;
                            best_vector.y = checked_coords[k].first;
                            best_vector.shift_dir = prev_pair.first;
                            best_vector.error = error;
                            previous_step = k;
                        }
                    }
                }
            }
            
            // Brute force
            /*
            for (const auto& prev_pair : prev_map) {
                const auto prev = prev_pair.second + vert_offset + hor_offset;
                for (int y = -BORDER; y <= BORDER; ++y) {
                    for (int x = -BORDER; x <= BORDER; ++x) {
                        const auto comp = prev + y * width_ext + x;
                        const int error = GetErrorSAD_16x16(cur, comp, width_ext);
                        if (error < best_vector.error) {
                            best_vector.x = x;
                            best_vector.y = y;
                            best_vector.shift_dir = prev_pair.first;
                            best_vector.error = error;
                        }
                    }
                }
            }
            */
            // Split into four subvectors if the error is too large
            if (best_vector.error > FIRST_SPLIT_ERROR) {
                best_vector.Split();

                for (int h = 0; h < 4; ++h) {
                    auto& subvector = best_vector.SubVector(h);
                    subvector.error = std::numeric_limits<long>::max();

                    const auto hor_offset = j * BLOCK_SIZE + ((h & 1) ? BLOCK_SIZE / 2 : 0);
                    const auto vert_offset = first_row_offset + (i * BLOCK_SIZE + ((h > 1) ? BLOCK_SIZE / 2 : 0)) * width_ext;
                    const auto cur = cur_Y + vert_offset + hor_offset;

                    int first_step = 0;
                    int points_checked = 8;
                    uint8_t ordered_tops[points_checked];
                    //SortTops(ordered_tops, &probabilities[i*num_blocks_hor*points_checked+j*points_checked]);

                    for (const auto& prev_pair : prev_map) {
                        const auto prev = prev_pair.second + vert_offset + hor_offset;
                        
                        int previous_step = 0;
                        std::pair<int, int> checked_coords[points_checked];
                        checked_coords[0] = std::make_pair(STEP, 0); // up
                        checked_coords[1] = std::make_pair(STEP, STEP);
                        checked_coords[2] = std::make_pair(0, STEP); // right
                        checked_coords[3] = std::make_pair(0-STEP, STEP);
                        checked_coords[4] = std::make_pair(0-STEP, 0); // down
                        checked_coords[5] = std::make_pair(0-STEP, 0-STEP);
                        checked_coords[6] = std::make_pair(0, 0-STEP); // left
                        checked_coords[7] = std::make_pair(STEP, 0-STEP);

                        for (int first_step = 0; first_step < points_checked; ++first_step)
                        {
                            const auto comp = prev + checked_coords[first_step].first * width_ext + checked_coords[first_step].second;
                            const int error = GetErrorSAD_8x8(cur, comp, width_ext);
                            //std::cout << "8x8 " << first_step << " " << error << std::endl;
                            if (error < subvector.error)
                            {
                                subvector.x = checked_coords[first_step].second;
                                subvector.y = checked_coords[first_step].first;
                                subvector.shift_dir = prev_pair.first;
                                subvector.error = error;
                                previous_step = first_step;
                            }
                            /*
                            if (error < PROB_ERROR_8)
                            {
                                //std::cout << "8x8 " << first_step << " " << error << std::endl;
                                break;
                            }
                            */
                        }
                        ++probabilities[i*num_blocks_hor*points_checked+j*points_checked+ordered_tops[first_step]];

                        for (int step = int(STEP/2); step > 0; step=int(step/2))
                        {
                            int prev_x = checked_coords[previous_step].second;
                            int prev_y = checked_coords[previous_step].first;
                            checked_coords[0] = std::make_pair(prev_y+step, prev_x); // up
                            checked_coords[1] = std::make_pair(prev_y+step, prev_x+step);
                            checked_coords[2] = std::make_pair(prev_y, prev_x+step); // right
                            checked_coords[3] = std::make_pair(prev_y-step, prev_x+step);
                            checked_coords[4] = std::make_pair(prev_y-step, prev_x); // down
                            checked_coords[5] = std::make_pair(prev_y-step, prev_x-step);
                            checked_coords[6] = std::make_pair(prev_y, prev_x-step); // left
                            checked_coords[7] = std::make_pair(prev_y+step, prev_x-step);
                            for (int k = 0; k < points_checked; ++k)
                            {
                                const auto comp = prev + checked_coords[k].first * width_ext + checked_coords[k].second;
                                const int error = GetErrorSAD_8x8(cur, comp, width_ext);
                                if (error < subvector.error)
                                {
                                    subvector.x = checked_coords[k].second;
                                    subvector.y = checked_coords[k].first;
                                    subvector.shift_dir = prev_pair.first;
                                    subvector.error = error;
                                    previous_step = k;
                                }
                            }    
                        }
                    }
                        
                    // PUT YOUR CODE HERE
                    /*
                    for (const auto& prev_pair : prev_map) {
                        const auto prev = prev_pair.second + vert_offset + hor_offset;
                        for (int y = -BORDER; y <= BORDER; ++y) {
                            for (int x = -BORDER; x <= BORDER; ++x) {
                                const auto comp = prev + y * width_ext + x;
                                const int error = GetErrorSAD_8x8(cur, comp, width_ext);

                                if (error < subvector.error) {
                                    subvector.x = x;
                                    subvector.y = y;
                                    subvector.shift_dir = prev_pair.first;
                                    subvector.error = error;
                                }
                            }
                        }
                    }
                    */
                }

                if (best_vector.SubVector(0).error
                    + best_vector.SubVector(1).error
                    + best_vector.SubVector(2).error
                    + best_vector.SubVector(3).error > best_vector.error * 0.7
                ) {
                    best_vector.Unsplit();
                }
            }
            mvectors.set_mv(j, i, best_vector);
        }
    }
}

void extend_with_borders(
    unsigned char *input,
    unsigned char *output,
    size_t height,
    size_t width,
    size_t border_size
) {
    // Copy frame to center of new 
    size_t new_width = width + 2 * border_size;
    auto p_output = output + new_width * border_size + border_size;
    auto p_input = input;
    for (size_t y = 0; y < height; ++y, p_output += 2 * border_size) {
        for (size_t x = 0; x < width; ++x, ++p_output, ++p_input) {
            *p_output = *p_input;
        }
    }

    // Left and right borders.
    p_output = output + new_width * border_size;
    for (size_t y = 0; y < height; ++y) {
        memset(p_output, p_output[border_size], border_size);
        p_output += border_size + width;
        memset(p_output, p_output[-1], border_size);
        p_output += border_size;
    }

    // Top and bottom borders.
    p_output = output;
    auto p_output_reference_row = p_output + new_width * border_size;

    for (size_t y = 0; y < border_size; ++y) {
        memcpy(p_output, p_output_reference_row, new_width);
        p_output += new_width;
    }
    p_output = output + new_width * (height + border_size);
    p_output_reference_row = p_output_reference_row - new_width;

    for (size_t y = 0; y < border_size; ++y) {
        memcpy(p_output, p_output_reference_row, new_width);
        p_output += new_width;
    }
}

void generate_subpixel_arrays(
    unsigned char* input,
    unsigned char* output_up,
    unsigned char* output_left,
    unsigned char* output_up_left,
    size_t height,
    size_t width
) {
    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            size_t cur_pixel_pos = y * width + x;
            size_t left_pixel_pos = y * width + x - 1;
            size_t left_up_pixel_pos = (y - 1) * width + x - 1;
            size_t up_pixel_pos = (y - 1) * width + x;
            
            if (y > 0) {
                output_up[cur_pixel_pos] = (int(input[cur_pixel_pos]) + input[up_pixel_pos]) / 2;
            } else {
                output_up[cur_pixel_pos] = input[cur_pixel_pos];
            }
            if (x > 0) {
                output_left[cur_pixel_pos] = (int(input[cur_pixel_pos]) + input[left_pixel_pos]) / 2;
            } else {
                output_left[cur_pixel_pos] = input[cur_pixel_pos];
            }

            if (x > 0 && y > 0) {   
                output_up_left[cur_pixel_pos] = (
                        int(input[cur_pixel_pos]) + 
                        input[left_pixel_pos] + 
                        input[left_up_pixel_pos] + 
                        input[up_pixel_pos]
                ) / 4;
            } else if (y == 0) {
                output_up_left[cur_pixel_pos] = output_left[cur_pixel_pos];
            } else {
                output_up_left[cur_pixel_pos] = output_up[cur_pixel_pos];
            }
        }
    }   
}

MEField MotionEstimator::Estimate(
    py::array_t<unsigned char> cur_Y,
    py::array_t<unsigned char> prev_Y,
    py::array_t<uint32_t, py::array::c_style> probabilities
) {
    
    extend_with_borders((unsigned char *)cur_Y.request().ptr, cur_Y_borders, height, width, BORDER);
    extend_with_borders((unsigned char *)prev_Y.request().ptr, prev_Y_borders, height, width, BORDER);
    
    if (cur_Y.size() != prev_Y.size()){
        throw std::runtime_error("Input shapes must match");
    }
    
    if (use_half_pixel) {
        generate_subpixel_arrays(
            prev_Y_borders,
            prev_Y_up_borders,
            prev_Y_left_borders,
            prev_Y_up_left_borders,
            width_borders,
            height_borders
        );
    }

    MotionEstimator::CEstimate(
        cur_Y_borders,
        prev_Y_borders,
        prev_Y_up_borders,
        prev_Y_left_borders,
        prev_Y_up_left_borders,
        (uint32_t*)probabilities.request().ptr,
        me_field
    );

    return me_field;
    
}
