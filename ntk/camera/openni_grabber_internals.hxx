/**
 * Copyright (C) 2013 ManCTL SARL <contact@manctl.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Burrus <nicolas.burrus@manctl.com>
 */

#define AVG(a,b) (((int)(a) + (int)(b)) >> 1)
#define AVG3(a,b,c) (((int)(a) + (int)(b) + (int)(c)) / 3)
#define AVG4(a,b,c,d) (((int)(a) + (int)(b) + (int)(c) + (int)(d)) >> 2)
#define WAVG4(a,b,c,d,x,y)  ( ( ((int)(a) + (int)(b)) * (int)(x) + ((int)(c) + (int)(d)) * (int)(y) ) / ( 2 * ((int)(x) + (int(y))) ) )

namespace
{

/**
 * @brief This class provides methods to fill a RGB or Grayscale image buffer from underlying Bayer pattern image.
 * @author Suat Gedikli
 * @date 02.january 2011
 * Note: imported from ROS (BSD License).
 */
class ImageBayerGRBG
{
public:
  typedef enum
  {
    Bilinear = 0,
    EdgeAware,
    EdgeAwareWeighted
  } DebayeringMethod;

  ImageBayerGRBG (DebayeringMethod method)
    : debayering_method_(method)
  {

  }

  void fillRGB (xn::ImageMetaData& xn_image,
                unsigned width, unsigned height,
                unsigned char* rgb_buffer,
                unsigned rgb_line_step = 0)
  {
    if (width > xn_image.XRes () || height > xn_image.YRes ())
      ntk_throw_exception(format("Upsampling only possible for multiple of 2 in both dimensions. Request was %d x %d -> %d x %d.", xn_image.XRes (), xn_image.YRes (), width, height));

    if (rgb_line_step == 0)
      rgb_line_step = width * 3;

    // padding skip for destination image
    unsigned rgb_line_skip = rgb_line_step - width * 3;

    if (xn_image.XRes () == width && xn_image.YRes () == height)
    {
      register const XnUInt8 *bayer_pixel = xn_image.Data ();
      register unsigned yIdx, xIdx;

      int bayer_line_step = xn_image.XRes ();
      int bayer_line_step2 = xn_image.XRes () << 1;

      if (debayering_method_ == Bilinear)
      {
        // first two pixel values for first two lines
        // Bayer         0 1 2
        //         0     G r g
        // line_step     b g b
        // line_step2    g r g

        rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
        rgb_buffer[1] = bayer_pixel[0]; // green pixel
        rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

        // Bayer         0 1 2
        //         0     g R g
        // line_step     b g b
        // line_step2    g r g
        //rgb_pixel[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
        rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

        // BGBG line
        // Bayer         0 1 2
        //         0     g r g
        // line_step     B g b
        // line_step2    g r g
        rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
        rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
        //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // pixel (1, 1)  0 1 2
        //         0     g r g
        // line_step     b G b
        // line_step2    g r g
        //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

        rgb_buffer += 6;
        bayer_pixel += 2;
        // rest of the first two lines

        for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
        {
          // GRGR line
          // Bayer        -1 0 1 2
          //           0   r G r g
          //   line_step   g b g b
          // line_step2    r g r g
          rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
          rgb_buffer[1] = bayer_pixel[0];
          rgb_buffer[2] = bayer_pixel[bayer_line_step + 1];

          // Bayer        -1 0 1 2
          //          0    r g R g
          //  line_step    g b g b
          // line_step2    r g r g
          rgb_buffer[3] = bayer_pixel[1];
          rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
          rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

          // BGBG line
          // Bayer         -1 0 1 2
          //         0      r g r g
          // line_step      g B g b
          // line_step2     r g r g
          rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
          rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
          rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

          // Bayer         -1 0 1 2
          //         0      r g r g
          // line_step      g b G b
          // line_step2     r g r g
          rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
          rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
          //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
        }

        // last two pixel values for first two lines
        // GRGR line
        // Bayer        -1 0 1
        //           0   r G r
        //   line_step   g b g
        // line_step2    r g r
        rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
        rgb_buffer[1] = bayer_pixel[0];
        rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

        // Bayer        -1 0 1
        //          0    r g R
        //  line_step    g b g
        // line_step2    r g r
        rgb_buffer[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
        //rgb_pixel[5] = bayer_pixel[line_step];

        // BGBG line
        // Bayer        -1 0 1
        //          0    r g r
        //  line_step    g B g
        // line_step2    r g r
        rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
        rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
        //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1
        //         0      r g r
        // line_step      g b G
        // line_step2     r g r
        rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

        bayer_pixel += bayer_line_step + 2;
        rgb_buffer += rgb_line_step + 6 + rgb_line_skip;

        // main processing

        for (yIdx = 2; yIdx < height - 2; yIdx += 2)
        {
          // first two pixel values
          // Bayer         0 1 2
          //        -1     b g b
          //         0     G r g
          // line_step     b g b
          // line_step2    g r g

          rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
          rgb_buffer[1] = bayer_pixel[0]; // green pixel
          rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]); // blue;

          // Bayer         0 1 2
          //        -1     b g b
          //         0     g R g
          // line_step     b g b
          // line_step2    g r g
          //rgb_pixel[3] = bayer_pixel[1];
          rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
          rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

          // BGBG line
          // Bayer         0 1 2
          //         0     g r g
          // line_step     B g b
          // line_step2    g r g
          rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
          rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
          rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

          // pixel (1, 1)  0 1 2
          //         0     g r g
          // line_step     b G b
          // line_step2    g r g
          //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
          rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
          rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

          rgb_buffer += 6;
          bayer_pixel += 2;
          // continue with rest of the line
          for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
          {
            // GRGR line
            // Bayer        -1 0 1 2
            //          -1   g b g b
            //           0   r G r g
            //   line_step   g b g b
            // line_step2    r g r g
            rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
            rgb_buffer[1] = bayer_pixel[0];
            rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

            // Bayer        -1 0 1 2
            //          -1   g b g b
            //          0    r g R g
            //  line_step    g b g b
            // line_step2    r g r g
            rgb_buffer[3] = bayer_pixel[1];
            rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
            rgb_buffer[5] = AVG4 (bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step], bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

            // BGBG line
            // Bayer         -1 0 1 2
            //         -1     g b g b
            //          0     r g r g
            // line_step      g B g b
            // line_step2     r g r g
            rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
            rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
            rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

            // Bayer         -1 0 1 2
            //         -1     g b g b
            //          0     r g r g
            // line_step      g b G b
            // line_step2     r g r g
            rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
            rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
            rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
          }

          // last two pixels of the line
          // last two pixel values for first two lines
          // GRGR line
          // Bayer        -1 0 1
          //           0   r G r
          //   line_step   g b g
          // line_step2    r g r
          rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
          rgb_buffer[1] = bayer_pixel[0];
          rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

          // Bayer        -1 0 1
          //          0    r g R
          //  line_step    g b g
          // line_step2    r g r
          rgb_buffer[3] = bayer_pixel[1];
          rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
          //rgb_pixel[5] = bayer_pixel[line_step];

          // BGBG line
          // Bayer        -1 0 1
          //          0    r g r
          //  line_step    g B g
          // line_step2    r g r
          rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
          rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
          //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

          // Bayer         -1 0 1
          //         0      r g r
          // line_step      g b G
          // line_step2     r g r
          rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
          rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
          //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

          bayer_pixel += bayer_line_step + 2;
          rgb_buffer += rgb_line_step + 6 + rgb_line_skip;
        }

        //last two lines
        // Bayer         0 1 2
        //        -1     b g b
        //         0     G r g
        // line_step     b g b

        rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
        rgb_buffer[1] = bayer_pixel[0]; // green pixel
        rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

        // Bayer         0 1 2
        //        -1     b g b
        //         0     g R g
        // line_step     b g b
        //rgb_pixel[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
        rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

        // BGBG line
        // Bayer         0 1 2
        //        -1     b g b
        //         0     g r g
        // line_step     B g b
        //rgb_pixel[rgb_line_step    ] = bayer_pixel[1];
        rgb_buffer[rgb_line_step + 1] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
        rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

        // Bayer         0 1 2
        //        -1     b g b
        //         0     g r g
        // line_step     b G b
        //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

        rgb_buffer += 6;
        bayer_pixel += 2;
        // rest of the last two lines
        for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
        {
          // GRGR line
          // Bayer       -1 0 1 2
          //        -1    g b g b
          //         0    r G r g
          // line_step    g b g b
          rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
          rgb_buffer[1] = bayer_pixel[0];
          rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

          // Bayer       -1 0 1 2
          //        -1    g b g b
          //         0    r g R g
          // line_step    g b g b
          rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
          rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
          rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[-bayer_line_step + 2]);

          // BGBG line
          // Bayer       -1 0 1 2
          //        -1    g b g b
          //         0    r g r g
          // line_step    g B g b
          rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[-1], bayer_pixel[1]);
          rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
          rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];


          // Bayer       -1 0 1 2
          //        -1    g b g b
          //         0    r g r g
          // line_step    g b G b
          //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
          rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
          rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
        }

        // last two pixel values for first two lines
        // GRGR line
        // Bayer       -1 0 1
        //        -1    g b g
        //         0    r G r
        // line_step    g b g
        rgb_buffer[rgb_line_step ] = rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
        rgb_buffer[1] = bayer_pixel[0];
        rgb_buffer[5] = rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

        // Bayer       -1 0 1
        //        -1    g b g
        //         0    r g R
        // line_step    g b g
        rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[-bayer_line_step + 1]);
        //rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

        // BGBG line
        // Bayer       -1 0 1
        //        -1    g b g
        //         0    r g r
        // line_step    g B g
        //rgb_pixel[rgb_line_step    ] = AVG2( bayer_pixel[-1], bayer_pixel[1] );
        rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
        rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

        // Bayer       -1 0 1
        //        -1    g b g
        //         0    r g r
        // line_step    g b G
        //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
      }
      else if (debayering_method_ == EdgeAware)
      {
        int dh, dv;

        // first two pixel values for first two lines
        // Bayer         0 1 2
        //         0     G r g
        // line_step     b g b
        // line_step2    g r g

        rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
        rgb_buffer[1] = bayer_pixel[0]; // green pixel
        rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

        // Bayer         0 1 2
        //         0     g R g
        // line_step     b g b
        // line_step2    g r g
        //rgb_pixel[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
        rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

        // BGBG line
        // Bayer         0 1 2
        //         0     g r g
        // line_step     B g b
        // line_step2    g r g
        rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
        rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
        //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // pixel (1, 1)  0 1 2
        //         0     g r g
        // line_step     b G b
        // line_step2    g r g
        //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

        rgb_buffer += 6;
        bayer_pixel += 2;
        // rest of the first two lines
        for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
        {
          // GRGR line
          // Bayer        -1 0 1 2
          //           0   r G r g
          //   line_step   g b g b
          // line_step2    r g r g
          rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
          rgb_buffer[1] = bayer_pixel[0];
          rgb_buffer[2] = bayer_pixel[bayer_line_step + 1];

          // Bayer        -1 0 1 2
          //          0    r g R g
          //  line_step    g b g b
          // line_step2    r g r g
          rgb_buffer[3] = bayer_pixel[1];
          rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
          rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

          // BGBG line
          // Bayer         -1 0 1 2
          //         0      r g r g
          // line_step      g B g b
          // line_step2     r g r g
          rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
          rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
          rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

          // Bayer         -1 0 1 2
          //         0      r g r g
          // line_step      g b G b
          // line_step2     r g r g
          rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
          rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
          //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
        }

        // last two pixel values for first two lines
        // GRGR line
        // Bayer        -1 0 1
        //           0   r G r
        //   line_step   g b g
        // line_step2    r g r
        rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
        rgb_buffer[1] = bayer_pixel[0];
        rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

        // Bayer        -1 0 1
        //          0    r g R
        //  line_step    g b g
        // line_step2    r g r
        rgb_buffer[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
        //rgb_pixel[5] = bayer_pixel[line_step];

        // BGBG line
        // Bayer        -1 0 1
        //          0    r g r
        //  line_step    g B g
        // line_step2    r g r
        rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
        rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
        //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1
        //         0      r g r
        // line_step      g b G
        // line_step2     r g r
        rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

        bayer_pixel += bayer_line_step + 2;
        rgb_buffer += rgb_line_step + 6 + rgb_line_skip;
        // main processing
        for (yIdx = 2; yIdx < height - 2; yIdx += 2)
        {
          // first two pixel values
          // Bayer         0 1 2
          //        -1     b g b
          //         0     G r g
          // line_step     b g b
          // line_step2    g r g

          rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
          rgb_buffer[1] = bayer_pixel[0]; // green pixel
          rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]); // blue;

          // Bayer         0 1 2
          //        -1     b g b
          //         0     g R g
          // line_step     b g b
          // line_step2    g r g
          //rgb_pixel[3] = bayer_pixel[1];
          rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
          rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

          // BGBG line
          // Bayer         0 1 2
          //         0     g r g
          // line_step     B g b
          // line_step2    g r g
          rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
          rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
          rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

          // pixel (1, 1)  0 1 2
          //         0     g r g
          // line_step     b G b
          // line_step2    g r g
          //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
          rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
          rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

          rgb_buffer += 6;
          bayer_pixel += 2;
          // continue with rest of the line
          for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
          {
            // GRGR line
            // Bayer        -1 0 1 2
            //          -1   g b g b
            //           0   r G r g
            //   line_step   g b g b
            // line_step2    r g r g
            rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
            rgb_buffer[1] = bayer_pixel[0];
            rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

            // Bayer        -1 0 1 2
            //          -1   g b g b
            //          0    r g R g
            //  line_step    g b g b
            // line_step2    r g r g

            dh = abs (bayer_pixel[0] - bayer_pixel[2]);
            dv = abs (bayer_pixel[-bayer_line_step + 1] - bayer_pixel[bayer_line_step + 1]);

            if (dh > dv)
              rgb_buffer[4] = AVG (bayer_pixel[-bayer_line_step + 1], bayer_pixel[bayer_line_step + 1]);
            else if (dv > dh)
              rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[2]);
            else
              rgb_buffer[4] = AVG4 (bayer_pixel[-bayer_line_step + 1], bayer_pixel[bayer_line_step + 1], bayer_pixel[0], bayer_pixel[2]);

            rgb_buffer[3] = bayer_pixel[1];
            rgb_buffer[5] = AVG4 (bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step], bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

            // BGBG line
            // Bayer         -1 0 1 2
            //         -1     g b g b
            //          0     r g r g
            // line_step      g B g b
            // line_step2     r g r g
            rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
            rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

            dv = abs (bayer_pixel[0] - bayer_pixel[bayer_line_step2]);
            dh = abs (bayer_pixel[bayer_line_step - 1] - bayer_pixel[bayer_line_step + 1]);

            if (dv > dh)
              rgb_buffer[rgb_line_step + 1] = AVG (bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
            else if (dh > dv)
              rgb_buffer[rgb_line_step + 1] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step2]);
            else
              rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);

            // Bayer         -1 0 1 2
            //         -1     g b g b
            //          0     r g r g
            // line_step      g b G b
            // line_step2     r g r g
            rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
            rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
            rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
          }

          // last two pixels of the line
          // last two pixel values for first two lines
          // GRGR line
          // Bayer        -1 0 1
          //           0   r G r
          //   line_step   g b g
          // line_step2    r g r
          rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
          rgb_buffer[1] = bayer_pixel[0];
          rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

          // Bayer        -1 0 1
          //          0    r g R
          //  line_step    g b g
          // line_step2    r g r
          rgb_buffer[3] = bayer_pixel[1];
          rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
          //rgb_pixel[5] = bayer_pixel[line_step];

          // BGBG line
          // Bayer        -1 0 1
          //          0    r g r
          //  line_step    g B g
          // line_step2    r g r
          rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
          rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
          //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

          // Bayer         -1 0 1
          //         0      r g r
          // line_step      g b G
          // line_step2     r g r
          rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
          rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
          //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

          bayer_pixel += bayer_line_step + 2;
          rgb_buffer += rgb_line_step + 6 + rgb_line_skip;
        }

        //last two lines
        // Bayer         0 1 2
        //        -1     b g b
        //         0     G r g
        // line_step     b g b

        rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
        rgb_buffer[1] = bayer_pixel[0]; // green pixel
        rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

        // Bayer         0 1 2
        //        -1     b g b
        //         0     g R g
        // line_step     b g b
        //rgb_pixel[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
        rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

        // BGBG line
        // Bayer         0 1 2
        //        -1     b g b
        //         0     g r g
        // line_step     B g b
        //rgb_pixel[rgb_line_step    ] = bayer_pixel[1];
        rgb_buffer[rgb_line_step + 1] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
        rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

        // Bayer         0 1 2
        //        -1     b g b
        //         0     g r g
        // line_step     b G b
        //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

        rgb_buffer += 6;
        bayer_pixel += 2;
        // rest of the last two lines
        for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
        {
          // GRGR line
          // Bayer       -1 0 1 2
          //        -1    g b g b
          //         0    r G r g
          // line_step    g b g b
          rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
          rgb_buffer[1] = bayer_pixel[0];
          rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

          // Bayer       -1 0 1 2
          //        -1    g b g b
          //         0    r g R g
          // line_step    g b g b
          rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
          rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
          rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[-bayer_line_step + 2]);

          // BGBG line
          // Bayer       -1 0 1 2
          //        -1    g b g b
          //         0    r g r g
          // line_step    g B g b
          rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[-1], bayer_pixel[1]);
          rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
          rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];


          // Bayer       -1 0 1 2
          //        -1    g b g b
          //         0    r g r g
          // line_step    g b G b
          //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
          rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
          rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
        }

        // last two pixel values for first two lines
        // GRGR line
        // Bayer       -1 0 1
        //        -1    g b g
        //         0    r G r
        // line_step    g b g
        rgb_buffer[rgb_line_step ] = rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
        rgb_buffer[1] = bayer_pixel[0];
        rgb_buffer[5] = rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

        // Bayer       -1 0 1
        //        -1    g b g
        //         0    r g R
        // line_step    g b g
        rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[-bayer_line_step + 1]);
        //rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

        // BGBG line
        // Bayer       -1 0 1
        //        -1    g b g
        //         0    r g r
        // line_step    g B g
        //rgb_pixel[rgb_line_step    ] = AVG2( bayer_pixel[-1], bayer_pixel[1] );
        rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
        rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

        // Bayer       -1 0 1
        //        -1    g b g
        //         0    r g r
        // line_step    g b G
        //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
      }
      else if (debayering_method_ == EdgeAwareWeighted)
      {
        int dh, dv;

        // first two pixel values for first two lines
        // Bayer         0 1 2
        //         0     G r g
        // line_step     b g b
        // line_step2    g r g

        rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
        rgb_buffer[1] = bayer_pixel[0]; // green pixel
        rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

        // Bayer         0 1 2
        //         0     g R g
        // line_step     b g b
        // line_step2    g r g
        //rgb_pixel[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
        rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

        // BGBG line
        // Bayer         0 1 2
        //         0     g r g
        // line_step     B g b
        // line_step2    g r g
        rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
        rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
        //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // pixel (1, 1)  0 1 2
        //         0     g r g
        // line_step     b G b
        // line_step2    g r g
        //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

        rgb_buffer += 6;
        bayer_pixel += 2;
        // rest of the first two lines
        for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
        {
          // GRGR line
          // Bayer        -1 0 1 2
          //           0   r G r g
          //   line_step   g b g b
          // line_step2    r g r g
          rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
          rgb_buffer[1] = bayer_pixel[0];
          rgb_buffer[2] = bayer_pixel[bayer_line_step + 1];

          // Bayer        -1 0 1 2
          //          0    r g R g
          //  line_step    g b g b
          // line_step2    r g r g
          rgb_buffer[3] = bayer_pixel[1];
          rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
          rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

          // BGBG line
          // Bayer         -1 0 1 2
          //         0      r g r g
          // line_step      g B g b
          // line_step2     r g r g
          rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
          rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
          rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

          // Bayer         -1 0 1 2
          //         0      r g r g
          // line_step      g b G b
          // line_step2     r g r g
          rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
          rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
          //rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
        }

        // last two pixel values for first two lines
        // GRGR line
        // Bayer        -1 0 1
        //           0   r G r
        //   line_step   g b g
        // line_step2    r g r
        rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
        rgb_buffer[1] = bayer_pixel[0];
        rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

        // Bayer        -1 0 1
        //          0    r g R
        //  line_step    g b g
        // line_step2    r g r
        rgb_buffer[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
        //rgb_pixel[5] = bayer_pixel[line_step];

        // BGBG line
        // Bayer        -1 0 1
        //          0    r g r
        //  line_step    g B g
        // line_step2    r g r
        rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
        rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
        //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

        // Bayer         -1 0 1
        //         0      r g r
        // line_step      g b G
        // line_step2     r g r
        rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

        bayer_pixel += bayer_line_step + 2;
        rgb_buffer += rgb_line_step + 6 + rgb_line_skip;
        // main processing
        for (yIdx = 2; yIdx < height - 2; yIdx += 2)
        {
          // first two pixel values
          // Bayer         0 1 2
          //        -1     b g b
          //         0     G r g
          // line_step     b g b
          // line_step2    g r g

          rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
          rgb_buffer[1] = bayer_pixel[0]; // green pixel
          rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]); // blue;

          // Bayer         0 1 2
          //        -1     b g b
          //         0     g R g
          // line_step     b g b
          // line_step2    g r g
          //rgb_pixel[3] = bayer_pixel[1];
          rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
          rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

          // BGBG line
          // Bayer         0 1 2
          //         0     g r g
          // line_step     B g b
          // line_step2    g r g
          rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
          rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
          rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

          // pixel (1, 1)  0 1 2
          //         0     g r g
          // line_step     b G b
          // line_step2    g r g
          //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
          rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
          rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

          rgb_buffer += 6;
          bayer_pixel += 2;
          // continue with rest of the line
          for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
          {
            // GRGR line
            // Bayer        -1 0 1 2
            //          -1   g b g b
            //           0   r G r g
            //   line_step   g b g b
            // line_step2    r g r g
            rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
            rgb_buffer[1] = bayer_pixel[0];
            rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

            // Bayer        -1 0 1 2
            //          -1   g b g b
            //          0    r g R g
            //  line_step    g b g b
            // line_step2    r g r g

            dh = abs (bayer_pixel[0] - bayer_pixel[2]);
            dv = abs (bayer_pixel[-bayer_line_step + 1] - bayer_pixel[bayer_line_step + 1]);

            if (dv == 0 && dh == 0)
              rgb_buffer[4] = AVG4 (bayer_pixel[1 - bayer_line_step], bayer_pixel[1 + bayer_line_step], bayer_pixel[0], bayer_pixel[2]);
            else
              rgb_buffer[4] = WAVG4 (bayer_pixel[1 - bayer_line_step], bayer_pixel[1 + bayer_line_step], bayer_pixel[0], bayer_pixel[2], dh, dv);
            rgb_buffer[3] = bayer_pixel[1];
            rgb_buffer[5] = AVG4 (bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step], bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

            // BGBG line
            // Bayer         -1 0 1 2
            //         -1     g b g b
            //          0     r g r g
            // line_step      g B g b
            // line_step2     r g r g
            rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
            rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

            dv = abs (bayer_pixel[0] - bayer_pixel[bayer_line_step2]);
            dh = abs (bayer_pixel[bayer_line_step - 1] - bayer_pixel[bayer_line_step + 1]);

            if (dv == 0 && dh == 0)
              rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
            else
              rgb_buffer[rgb_line_step + 1] = WAVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1], dh, dv);

            // Bayer         -1 0 1 2
            //         -1     g b g b
            //          0     r g r g
            // line_step      g b G b
            // line_step2     r g r g
            rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
            rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
            rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
          }

          // last two pixels of the line
          // last two pixel values for first two lines
          // GRGR line
          // Bayer        -1 0 1
          //           0   r G r
          //   line_step   g b g
          // line_step2    r g r
          rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
          rgb_buffer[1] = bayer_pixel[0];
          rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

          // Bayer        -1 0 1
          //          0    r g R
          //  line_step    g b g
          // line_step2    r g r
          rgb_buffer[3] = bayer_pixel[1];
          rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
          //rgb_pixel[5] = bayer_pixel[line_step];

          // BGBG line
          // Bayer        -1 0 1
          //          0    r g r
          //  line_step    g B g
          // line_step2    r g r
          rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
          rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
          //rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

          // Bayer         -1 0 1
          //         0      r g r
          // line_step      g b G
          // line_step2     r g r
          rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
          rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
          //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

          bayer_pixel += bayer_line_step + 2;
          rgb_buffer += rgb_line_step + 6 + rgb_line_skip;
        }

        //last two lines
        // Bayer         0 1 2
        //        -1     b g b
        //         0     G r g
        // line_step     b g b

        rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
        rgb_buffer[1] = bayer_pixel[0]; // green pixel
        rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

        // Bayer         0 1 2
        //        -1     b g b
        //         0     g R g
        // line_step     b g b
        //rgb_pixel[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
        rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

        // BGBG line
        // Bayer         0 1 2
        //        -1     b g b
        //         0     g r g
        // line_step     B g b
        //rgb_pixel[rgb_line_step    ] = bayer_pixel[1];
        rgb_buffer[rgb_line_step + 1] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
        rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

        // Bayer         0 1 2
        //        -1     b g b
        //         0     g r g
        // line_step     b G b
        //rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

        rgb_buffer += 6;
        bayer_pixel += 2;
        // rest of the last two lines
        for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
        {
          // GRGR line
          // Bayer       -1 0 1 2
          //        -1    g b g b
          //         0    r G r g
          // line_step    g b g b
          rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
          rgb_buffer[1] = bayer_pixel[0];
          rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

          // Bayer       -1 0 1 2
          //        -1    g b g b
          //         0    r g R g
          // line_step    g b g b
          rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
          rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
          rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[-bayer_line_step + 2]);

          // BGBG line
          // Bayer       -1 0 1 2
          //        -1    g b g b
          //         0    r g r g
          // line_step    g B g b
          rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[-1], bayer_pixel[1]);
          rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
          rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];


          // Bayer       -1 0 1 2
          //        -1    g b g b
          //         0    r g r g
          // line_step    g b G b
          //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
          rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
          rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
        }

        // last two pixel values for first two lines
        // GRGR line
        // Bayer       -1 0 1
        //        -1    g b g
        //         0    r G r
        // line_step    g b g
        rgb_buffer[rgb_line_step ] = rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
        rgb_buffer[1] = bayer_pixel[0];
        rgb_buffer[5] = rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

        // Bayer       -1 0 1
        //        -1    g b g
        //         0    r g R
        // line_step    g b g
        rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
        rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[-bayer_line_step + 1]);
        //rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

        // BGBG line
        // Bayer       -1 0 1
        //        -1    g b g
        //         0    r g r
        // line_step    g B g
        //rgb_pixel[rgb_line_step    ] = AVG2( bayer_pixel[-1], bayer_pixel[1] );
        rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
        rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

        // Bayer       -1 0 1
        //        -1    g b g
        //         0    r g r
        // line_step    g b G
        //rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
        rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
        //rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
      }
      else
        ntk_throw_exception(format("Unknwon debayering method: %d", (int)debayering_method_));
    }
    else
    {
      if (xn_image.XRes () % width != 0 || xn_image.YRes () % height != 0)
        ntk_throw_exception(format("Downsampling only possible for integer scales in both dimensions. Request was %d x %d -> %d x %d.", xn_image.XRes (), xn_image.YRes (), width, height));

      // get each or each 2nd pixel group to find rgb values!
      register unsigned bayerXStep = xn_image.XRes () / width;
      register unsigned bayerYSkip = (xn_image.YRes () / height - 1) * xn_image.XRes ();

      // Downsampling and debayering at once
      register const XnUInt8* bayer_buffer = xn_image.Data ();

      for (register unsigned yIdx = 0; yIdx < height; ++yIdx, bayer_buffer += bayerYSkip, rgb_buffer += rgb_line_skip) // skip a line
      {
        for (register unsigned xIdx = 0; xIdx < width; ++xIdx, rgb_buffer += 3, bayer_buffer += bayerXStep)
        {
          rgb_buffer[ 2 ] = bayer_buffer[ xn_image.XRes () ];
          rgb_buffer[ 1 ] = AVG (bayer_buffer[0], bayer_buffer[ xn_image.XRes () + 1]);
          rgb_buffer[ 0 ] = bayer_buffer[ 1 ];
        }
      }
    }
  }

protected:
  DebayeringMethod debayering_method_;
};

} // namespace
