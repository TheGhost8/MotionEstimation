#include "me_estimator.h"
#include "metric.h"
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

    probabilities_first = new uint32_t[num_blocks_vert*num_blocks_hor*DIRECTIONS]{1};
    probabilities_second = new uint32_t[num_blocks_vert*num_blocks_hor*WIDE_DIRECTIONS]{1};
    probabilities_third = new uint32_t[num_blocks_vert*num_blocks_hor*WIDE_DIRECTIONS]{1};


    switch (quality)
    {
        case 100:
            PROB_ERROR_16 = new int32_t[3]{1750, 900, 700};
            PROB_ERROR_8 = new int32_t[3]{700, 600, 500};
            break;
        case 80:
            PROB_ERROR_16 = new int32_t[3]{2000, 1100, 900};
            PROB_ERROR_8 = new int32_t[3]{900, 800, 700};
            break;
        case 60:
            PROB_ERROR_16 = new int32_t[3]{2300, 1200, 1000};
            PROB_ERROR_8 = new int32_t[3]{900, 800, 700};
            break;
        case 40:
            PROB_ERROR_16 = new int32_t[3]{3000, 2000, 1500};
            PROB_ERROR_8 = new int32_t[3]{1000, 900, 800};
            break;
        case 20:
            PROB_ERROR_16 = new int32_t[3]{6000, 4000, 2500};
            PROB_ERROR_8 = new int32_t[3]{1100, 1000, 900};
            break;
        default:
            PROB_ERROR_16 = new int32_t[3]{1750, 900, 700};
            PROB_ERROR_8 = new int32_t[3]{800, 700, 600};
            break;
    }
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

    delete[] probabilities_first;
    delete[] probabilities_second;
    delete[] probabilities_third;
    delete[] PROB_ERROR_16;
    delete[] PROB_ERROR_8;
}

void SortTops(uint8_t* tops, const uint32_t* probabilities, int length)
{
    int third;
    for (int i = 0; i < length; ++i)
    {
        if (probabilities[tops[i]] == 1)
        {
            break;
        }
        for (int j = i+1; j < length; ++j)
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
            
            int first_step = 0;
            int previous_step = 0;
            int points_checked = 8;
            std::pair<int, int> checked_coords[points_checked];
            uint8_t ordered_first_tops[DIRECTIONS]{0,1,2,3};
            uint8_t ordered_second_tops[WIDE_DIRECTIONS]{0,1,2,3,4,5,6,7};
            uint8_t start_indexes[WIDE_DIRECTIONS]{0,1,2,3,4,5,6,7};
            SortTops(ordered_first_tops, &probabilities_first[i*num_blocks_hor*DIRECTIONS+j*DIRECTIONS], DIRECTIONS);

            for (const auto& prev_pair : prev_map)
            {
                const auto prev = prev_pair.second + vert_offset + hor_offset;

                points_checked = 4;
                checked_coords[0] = std::make_pair(STEP, 0); // up
                checked_coords[1] = std::make_pair(0, STEP); // right
                checked_coords[2] = std::make_pair(0-STEP, 0); // down
                checked_coords[3] = std::make_pair(0, 0-STEP); // left

                for (first_step = 0; first_step < points_checked; ++first_step)
                {
                    const auto comp = prev + checked_coords[ordered_first_tops[first_step]].first * width_ext + checked_coords[ordered_first_tops[first_step]].second;
                    const int error = GetErrorSAD_16x16(cur, comp, width_ext);
                    if (error < best_vector.error)
                    {
                        best_vector.x = checked_coords[ordered_first_tops[first_step]].second;
                        best_vector.y = checked_coords[ordered_first_tops[first_step]].first;
                        best_vector.shift_dir = prev_pair.first;
                        best_vector.error = error;
                        previous_step = ordered_first_tops[first_step];
                    }
                    if (error < PROB_ERROR_16[0])
                    {
                        //std::cout << "16x16  first_step " << first_step << " " << error << " " << PROB_ERROR_16[0] << std::endl;
                        break;
                    }
                }
                ++probabilities_first[i*num_blocks_hor*DIRECTIONS+j*DIRECTIONS+ordered_first_tops[first_step]];

                for (int step = int(STEP/2); step > 0; step=int(step/2))
                {
                    std::copy(start_indexes, start_indexes + WIDE_DIRECTIONS, ordered_second_tops);
                    if (step == int(STEP/2))
                    {
                        SortTops(ordered_second_tops, &probabilities_second[i*num_blocks_hor*WIDE_DIRECTIONS+j*WIDE_DIRECTIONS], WIDE_DIRECTIONS);
                    }
                    else
                    {
                        SortTops(ordered_second_tops, &probabilities_third[i*num_blocks_hor*WIDE_DIRECTIONS+j*WIDE_DIRECTIONS], WIDE_DIRECTIONS);
                    }

                    points_checked = 8;
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
                        const auto comp = prev + checked_coords[ordered_second_tops[k]].first * width_ext + checked_coords[ordered_second_tops[k]].second;
                        const int error = GetErrorSAD_16x16(cur, comp, width_ext);
                        if (error < best_vector.error)
                        {
                            best_vector.x = checked_coords[ordered_second_tops[k]].second;
                            best_vector.y = checked_coords[ordered_second_tops[k]].first;
                            best_vector.shift_dir = prev_pair.first;
                            best_vector.error = error;
                            previous_step = ordered_second_tops[k];
                        }
                        if ((step == int(STEP/2)) && (error < PROB_ERROR_16[1]))
                        {
                            //std::cout << "16x16 second_step " << k << " " << error << " " << PROB_ERROR_16[1] << std::endl;
                            break;
                        }
                        else if (error < PROB_ERROR_16[2])
                        {
                            //std::cout << "16x16  third_step " << k << " " << error << " " << PROB_ERROR_16[2] << std::endl;
                            break;
                        }
                    }

                    if (step == int(STEP/2))
                    {
                        ++probabilities_second[i*num_blocks_hor*DIRECTIONS+j*DIRECTIONS+previous_step];
                    }
                    else
                    {
                        ++probabilities_third[i*num_blocks_hor*DIRECTIONS+j*DIRECTIONS+previous_step];
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
                    int previous_step = 0;
                    int points_checked = 8;
                    std::pair<int, int> checked_coords[points_checked];
                    uint8_t ordered_first_tops[DIRECTIONS]{0,1,2,3};
                    uint8_t ordered_second_tops[WIDE_DIRECTIONS]{0,1,2,3,4,5,6,7};
                    uint8_t start_indexes[WIDE_DIRECTIONS]{0,1,2,3,4,5,6,7};
                    SortTops(ordered_first_tops, &probabilities_first[i*num_blocks_hor*DIRECTIONS+j*DIRECTIONS], DIRECTIONS);

                    for (const auto& prev_pair : prev_map)
                    {
                        const auto prev = prev_pair.second + vert_offset + hor_offset;

                        points_checked = 4;
                        checked_coords[0] = std::make_pair(STEP, 0); // up
                        checked_coords[1] = std::make_pair(0, STEP); // right
                        checked_coords[2] = std::make_pair(0-STEP, 0); // down
                        checked_coords[3] = std::make_pair(0, 0-STEP); // left

                        for (first_step = 0; first_step < points_checked; ++first_step)
                        {
                            const auto comp = prev + checked_coords[ordered_first_tops[first_step]].first * width_ext + checked_coords[ordered_first_tops[first_step]].second;
                            const int error = GetErrorSAD_8x8(cur, comp, width_ext);
                            if (error < subvector.error)
                            {
                                subvector.x = checked_coords[ordered_first_tops[first_step]].second;
                                subvector.y = checked_coords[ordered_first_tops[first_step]].first;
                                subvector.shift_dir = prev_pair.first;
                                subvector.error = error;
                                previous_step = ordered_first_tops[first_step];
                            }
                            if (error < PROB_ERROR_8[0])
                            {
                                //std::cout << " 8x8   first_step " << first_step << " " << error << " " << PROB_ERROR_8[0] << std::endl;
                                break;
                            }
                        }
                        ++probabilities_first[i*num_blocks_hor*DIRECTIONS+j*DIRECTIONS+ordered_first_tops[first_step]];

                        for (int step = int(STEP/2); step > 0; step=int(step/2))
                        {
                            std::copy(start_indexes, start_indexes + WIDE_DIRECTIONS, ordered_second_tops);
                            if (step == int(STEP/2))
                            {
                                SortTops(ordered_second_tops, &probabilities_second[i*num_blocks_hor*WIDE_DIRECTIONS+j*WIDE_DIRECTIONS], WIDE_DIRECTIONS);
                            }
                            else
                            {
                                SortTops(ordered_second_tops, &probabilities_third[i*num_blocks_hor*WIDE_DIRECTIONS+j*WIDE_DIRECTIONS], WIDE_DIRECTIONS);
                            }

                            points_checked = 8;
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
                                const auto comp = prev + checked_coords[ordered_second_tops[k]].first * width_ext + checked_coords[ordered_second_tops[k]].second;
                                const int error = GetErrorSAD_8x8(cur, comp, width_ext);
                                if (error < subvector.error)
                                {
                                    subvector.x = checked_coords[ordered_second_tops[k]].second;
                                    subvector.y = checked_coords[ordered_second_tops[k]].first;
                                    subvector.shift_dir = prev_pair.first;
                                    subvector.error = error;
                                    previous_step = ordered_second_tops[k];
                                }
                                if ((step == int(STEP/2)) && (error < PROB_ERROR_8[1]))
                                {
                                    //std::cout << " 8x8  second_step " << k << " " << error << " " << PROB_ERROR_8[1] << std::endl;
                                    break;
                                }
                                else if (error < PROB_ERROR_8[2])
                                {
                                    //std::cout << " 8x8   third_step " << k << " " << error << " " << PROB_ERROR_8[2] << std::endl;
                                    break;
                                }
                            }

                            if (step == int(STEP/2))
                            {
                                ++probabilities_second[i*num_blocks_hor*DIRECTIONS+j*DIRECTIONS+previous_step];
                            }
                            else
                            {
                                ++probabilities_third[i*num_blocks_hor*DIRECTIONS+j*DIRECTIONS+previous_step];
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
    py::array_t<unsigned char> prev_Y
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
        me_field
    );

    return me_field;
    
}
