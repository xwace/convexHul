//
// Created by star on 24-4-8.
//

#include <opencv2/imgproc/imgproc_c.h>
#include "fillConvec.h"
#include "opencv2/opencv.hpp"

using namespace cv;
/****************************************************************************************\
*                                Polygons filling                                        *
\****************************************************************************************/
enum { XY_SHIFT = 16, XY_ONE = 1 << XY_SHIFT, DRAWING_STORAGE_BLOCK = (1<<12) - 256 };
static void
Line2( Mat& img, Point2l pt1, Point2l pt2, const void* color)
{
    int64 dx, dy;
    int ecount;
    int64 ax, ay;
    int64 i, j;
    int x, y;
    int64 x_step, y_step;
    int cb = ((uchar*)color)[0];
    int cg = ((uchar*)color)[1];
    int cr = ((uchar*)color)[2];
    int pix_size = (int)img.elemSize();
    uchar *ptr = img.ptr(), *tptr;
    size_t step = img.step;
    Size size = img.size();

    //CV_Assert( img && (nch == 1 || nch == 3) && img.depth() == CV_8U );

    Size2l sizeScaled(((int64)size.width) << XY_SHIFT, ((int64)size.height) << XY_SHIFT);
    if( !clipLine( sizeScaled, pt1, pt2 ))
        return;

    dx = pt2.x - pt1.x;
    dy = pt2.y - pt1.y;

    j = dx < 0 ? -1 : 0;
    ax = (dx ^ j) - j;
    i = dy < 0 ? -1 : 0;
    ay = (dy ^ i) - i;

    if( ax > ay )
    {
        dy = (dy ^ j) - j;
        pt1.x ^= pt2.x & j;
        pt2.x ^= pt1.x & j;
        pt1.x ^= pt2.x & j;
        pt1.y ^= pt2.y & j;
        pt2.y ^= pt1.y & j;
        pt1.y ^= pt2.y & j;

        x_step = XY_ONE;
        y_step = (dy << XY_SHIFT) / (ax | 1);
        ecount = (int)((pt2.x - pt1.x) >> XY_SHIFT);
    }
    else
    {
        dx = (dx ^ i) - i;
        pt1.x ^= pt2.x & i;
        pt2.x ^= pt1.x & i;
        pt1.x ^= pt2.x & i;
        pt1.y ^= pt2.y & i;
        pt2.y ^= pt1.y & i;
        pt1.y ^= pt2.y & i;

        x_step = (dx << XY_SHIFT) / (ay | 1);
        y_step = XY_ONE;
        ecount = (int)((pt2.y - pt1.y) >> XY_SHIFT);
    }

    pt1.x += (XY_ONE >> 1);
    pt1.y += (XY_ONE >> 1);

    if( pix_size == 3 )
    {
#define  ICV_PUT_POINT(_x,_y)   \
        x = (_x); y = (_y);             \
        if( 0 <= x && x < size.width && \
            0 <= y && y < size.height ) \
        {                               \
            tptr = ptr + y*step + x*3;  \
            tptr[0] = (uchar)cb;        \
            tptr[1] = (uchar)cg;        \
            tptr[2] = (uchar)cr;        \
        }

        ICV_PUT_POINT((int)((pt2.x + (XY_ONE >> 1)) >> XY_SHIFT),
                      (int)((pt2.y + (XY_ONE >> 1)) >> XY_SHIFT));

        if( ax > ay )
        {
            pt1.x >>= XY_SHIFT;

            while( ecount >= 0 )
            {
                ICV_PUT_POINT((int)(pt1.x), (int)(pt1.y >> XY_SHIFT));
                pt1.x++;
                pt1.y += y_step;
                ecount--;
            }
        }
        else
        {
            pt1.y >>= XY_SHIFT;

            while( ecount >= 0 )
            {
                ICV_PUT_POINT((int)(pt1.x >> XY_SHIFT), (int)(pt1.y));
                pt1.x += x_step;
                pt1.y++;
                ecount--;
            }
        }

#undef ICV_PUT_POINT
    }
    else if( pix_size == 1 )
    {
#define  ICV_PUT_POINT(_x,_y) \
        x = (_x); y = (_y);           \
        if( 0 <= x && x < size.width && \
            0 <= y && y < size.height ) \
        {                           \
            tptr = ptr + y*step + x;\
            tptr[0] = (uchar)cb;    \
        }

        ICV_PUT_POINT((int)((pt2.x + (XY_ONE >> 1)) >> XY_SHIFT),
                      (int)((pt2.y + (XY_ONE >> 1)) >> XY_SHIFT));

        if( ax > ay )
        {
            pt1.x >>= XY_SHIFT;

            while( ecount >= 0 )
            {
                ICV_PUT_POINT((int)(pt1.x), (int)(pt1.y >> XY_SHIFT));
                pt1.x++;
                pt1.y += y_step;
                ecount--;
            }
        }
        else
        {
            pt1.y >>= XY_SHIFT;

            while( ecount >= 0 )
            {
                ICV_PUT_POINT((int)(pt1.x >> XY_SHIFT), (int)(pt1.y));
                pt1.x += x_step;
                pt1.y++;
                ecount--;
            }
        }

#undef ICV_PUT_POINT
    }
    else
    {
#define  ICV_PUT_POINT(_x,_y)   \
        x = (_x); y = (_y);             \
        if( 0 <= x && x < size.width && \
            0 <= y && y < size.height ) \
        {                               \
            tptr = ptr + y*step + x*pix_size;\
            for( j = 0; j < pix_size; j++ ) \
                tptr[j] = ((uchar*)color)[j]; \
        }

        ICV_PUT_POINT((int)((pt2.x + (XY_ONE >> 1)) >> XY_SHIFT),
                      (int)((pt2.y + (XY_ONE >> 1)) >> XY_SHIFT));

        if( ax > ay )
        {
            pt1.x >>= XY_SHIFT;

            while( ecount >= 0 )
            {
                ICV_PUT_POINT((int)(pt1.x), (int)(pt1.y >> XY_SHIFT));
                pt1.x++;
                pt1.y += y_step;
                ecount--;
            }
        }
        else
        {
            pt1.y >>= XY_SHIFT;

            while( ecount >= 0 )
            {
                ICV_PUT_POINT((int)(pt1.x >> XY_SHIFT), (int)(pt1.y));
                pt1.x += x_step;
                pt1.y++;
                ecount--;
            }
        }

#undef ICV_PUT_POINT
    }
}

static void
Line( Mat& img, Point pt1, Point pt2,
      const void* _color, int connectivity = 8 )
{
    if( connectivity == 0 )
        connectivity = 8;
    else if( connectivity == 1 )
        connectivity = 4;

    LineIterator iterator(img, pt1, pt2, connectivity, true);
    int i, count = iterator.count;
    int pix_size = (int)img.elemSize();
    const uchar* color = (const uchar*)_color;

    if( pix_size == 3 )
    {
        for( i = 0; i < count; i++, ++iterator )
        {
            uchar* ptr = *iterator;
            ptr[0] = color[0];
            ptr[1] = color[1];
            ptr[2] = color[2];
        }
    }
    else
    {
        for( i = 0; i < count; i++, ++iterator )
        {
            uchar* ptr = *iterator;
            if( pix_size == 1 )
                ptr[0] = color[0];
            else
                memcpy( *iterator, color, pix_size );
        }
    }
}


static inline void ICV_HLINE_X(uchar* ptr, int xl, int xr, const uchar* color, int pix_size)
{
    uchar* hline_min_ptr = (uchar*)(ptr) + (xl)*(pix_size);
    uchar* hline_end_ptr = (uchar*)(ptr) + (xr+1)*(pix_size);
    uchar* hline_ptr = hline_min_ptr;
    if (pix_size == 1)
        memset(hline_min_ptr, *color, hline_end_ptr-hline_min_ptr);
    else//if (pix_size != 1)
    {
        if (hline_min_ptr < hline_end_ptr)
        {
            memcpy(hline_ptr, color, pix_size);
            hline_ptr += pix_size;
        }//end if (hline_min_ptr < hline_end_ptr)
        size_t sizeToCopy = pix_size;
        while(hline_ptr < hline_end_ptr)
        {
            memcpy(hline_ptr, hline_min_ptr, sizeToCopy);
            hline_ptr += sizeToCopy;
            sizeToCopy = std::min(2*sizeToCopy, static_cast<size_t>(hline_end_ptr-hline_ptr));
        }//end while(hline_ptr < hline_end_ptr)
    }//end if (pix_size != 1)
}
//end ICV_HLINE_X()

static inline void ICV_HLINE(uchar* ptr, int xl, int xr, const void* color, int pix_size)
{
    ICV_HLINE_X(ptr, xl, xr, reinterpret_cast<const uchar*>(color), pix_size);
}
//end ICV_HLINE()

/* filling convex polygon. v - array of vertices, ntps - number of points */
void FillConvexPoly( Mat& img, const Point2l* v, int npts, const void* color, int line_type, int shift )
{
    struct
    {
        int idx, di;
        int64 x, dx;
        int ye;
    }edge[2];

    int delta = 1 << shift >> 1;
    int i, y, imin = 0;
    int edges = npts;
    int64 xmin, xmax, ymin, ymax;
    uchar* ptr = img.ptr();
    Size size = img.size();
    int pix_size = (int)img.elemSize();
    Point2l p0;
    int delta1, delta2;

    if( line_type < CV_AA )
        delta1 = delta2 = XY_ONE >> 1;
    else
        delta1 = XY_ONE - 1, delta2 = 0;

    p0 = v[npts - 1];
    p0.x <<= XY_SHIFT - shift;
    p0.y <<= XY_SHIFT - shift;

    CV_Assert( 0 <= shift && shift <= XY_SHIFT );
    xmin = xmax = v[0].x;
    ymin = ymax = v[0].y;

    for( i = 0; i < npts; i++ )
    {
        Point2l p = v[i];
        if( p.y < ymin )
        {
            ymin = p.y;
            imin = i;
        }

        ymax = std::max( ymax, p.y );
        xmax = std::max( xmax, p.x );
        xmin = MIN( xmin, p.x );

        p.x <<= XY_SHIFT - shift;
        p.y <<= XY_SHIFT - shift;

        if( line_type <= 8 )
        {
            if( shift == 0 )
            {
                Point pt0, pt1;
                pt0.x = (int)(p0.x >> XY_SHIFT);
                pt0.y = (int)(p0.y >> XY_SHIFT);
                pt1.x = (int)(p.x >> XY_SHIFT);
                pt1.y = (int)(p.y >> XY_SHIFT);
                Line( img, pt0, pt1, color, line_type );
            }
            else
                Line2( img, p0, p, color );
        }
        else
            std::cout<<"errrrrrr"<<std::endl;
            //LineAA( img, p0, p, color );
        p0 = p;
    }

    namedWindow("img",2);
    cv::imshow("img",img==5);
    cv::waitKey();

    xmin = (xmin + delta) >> shift;
    xmax = (xmax + delta) >> shift;
    ymin = (ymin + delta) >> shift;
    ymax = (ymax + delta) >> shift;

    if( npts < 3 || (int)xmax < 0 || (int)ymax < 0 || (int)xmin >= size.width || (int)ymin >= size.height )
        return;

    ymax = MIN( ymax, size.height - 1 );
    edge[0].idx = edge[1].idx = imin;

    edge[0].ye = edge[1].ye = y = (int)ymin;
    edge[0].di = 1;
    edge[1].di = npts - 1;

    edge[0].x = edge[1].x = -XY_ONE;
    edge[0].dx = edge[1].dx = 0;

    ptr += img.step*y;

    do
    {
        if( line_type < CV_AA || y < (int)ymax || y == (int)ymin )
        {
            for( i = 0; i < 2; i++ )
            {
                if( y >= edge[i].ye )
                {
                    int idx0 = edge[i].idx, di = edge[i].di;
                    int idx = idx0 + di;
                    if (idx >= npts) idx -= npts;
                    int ty = 0;

                    for (; edges-- > 0; )
                    {
                        ty = (int)((v[idx].y + delta) >> shift);
                        if (ty > y)
                        {
                            int64 xs = v[idx0].x;
                            int64 xe = v[idx].x;
                            if (shift != XY_SHIFT)
                            {
                                xs <<= XY_SHIFT - shift;
                                xe <<= XY_SHIFT - shift;
                            }

                            std::cout<<"ty-y: "<<ty<<" "<<y<<std::endl;

                            edge[i].ye = ty;
                            edge[i].dx = ((xe - xs)*2 + (ty - y)) / (2 * (ty - y));
                            edge[i].x = xs;
                            edge[i].idx = idx;
                            std::cout<<"ye: "<<ty<<" "<<(edge[i].dx>>XY_SHIFT)<<" "<<(xs>>XY_SHIFT)<<" idx: "<<idx<<std::endl;getchar();
                            break;
                        }

                        std::cout<<"eges: "<<edges<<std::endl;
                        idx0 = idx;
                        idx += di;
                        if (idx >= npts) idx -= npts;
                    }
                }
            }
        }

        if (edges < 0)
            break;

        if (y >= 0)
        {
            int left = 0, right = 1;
            if (edge[0].x > edge[1].x)
            {
                left = 1, right = 0;
            }

            int xx1 = (int)((edge[left].x + delta1) >> XY_SHIFT);
            int xx2 = (int)((edge[right].x + delta2) >> XY_SHIFT);

            if( xx2 >= 0 && xx1 < size.width )
            {
                if( xx1 < 0 )
                    xx1 = 0;
                if( xx2 >= size.width )
                    xx2 = size.width - 1;
                ICV_HLINE( ptr, xx1, xx2, color, pix_size );
            }
        }
        else
        {
            // TODO optimize scan for negative y
        }

        edge[0].x += edge[0].dx;
        edge[1].x += edge[1].dx;
        ptr += img.step;
        cv::imshow("img",img==5);
        cv::waitKey();
    }
    while( ++y <= (int)ymax );
}

int main(){

    cv::Mat img(200,200,0,cv::Scalar(0));
    std::vector<cv::Point2l>vec={{50,20},{30,30},{50,50},{70,25}};
    cv::Point2l* v = &vec[0];
    int npts = vec.size();
    int color = 5;
    FillConvexPoly( img, v, npts, &color, 8, 0);
    cv::imshow("img",img==5);
    cv::waitKey();
}

