#include <FL/fl_draw.H>
#include "Fl_MRender.H"

static char str_no_source[] = "NO SOURCE";

Fl_MRender::Fl_MRender(int X, int Y, int W, int H, const char *l)
    : Fl_Box(X,Y,W,H,l),
    _bdraw( true ),
    _putx( 0 ),
    _puty( 0 ),
    _putw( 0 ),
    _puth( 0 ),
    _ostr( str_no_source ),
    _eventh( NULL ),
    _cachedimg( NULL )
{
    box( FL_NO_BOX );
    color( FL_BLACK );
}

Fl_MRender::~Fl_MRender()
{

}

void  Fl_MRender::eventhandler( Fl_MRender_Event* p )
{
    _eventh = p;
}

void Fl_MRender::draw()
{
    fl_push_clip( x(), y(), w(), h() );
    //Fl_Box::draw();

    if ( _bdraw == true )
    {
        Fl_Image* refimg = _cachedimg;

        if ( refimg == NULL )
        {
            refimg = Fl_Box::image();
        }

        if ( ( refimg != NULL ) && ( _putw > 0 ) && ( _puth > 0 ) )
        {
            refimg->draw( _putx, _puty );
        }

    }

#ifdef DEBUG
    fl_color( FL_RED );
    fl_rect( _putx, _puty, _putw, _puth );
#endif

    fl_pop_clip();
}

void Fl_MRender::resize(int x, int y, int w, int h)
{
    Fl_Box::resize(x,y,w,h);

    if ( _isInResize == true )
        return;

    recalcimg( w, h );

#ifdef DEBUG
    printf("--- putting image resized as : %d x %d\n", _putw, _puth );
#endif

}

void Fl_MRender::image( Fl_Image* img )
{
    if ( _cachedimg != NULL )
    {
        delete _cachedimg;
        _cachedimg = NULL;
    }

    Fl_Box::image( img );

    if ( img != NULL )
    {
        _orgw = img->w();
        _orgh = img->h();

        _putx = 0;
        _puty = 0;
        _putw = _orgw;
        _puth = _putw;

        recalcimg( w(), h() );
    }
    else
    {
        _orgw = 0;
        _orgh = 0;

        _putx = 0;
        _puty = 0;
        _putw = 0;
        _puth = 0;
    }

}

void Fl_MRender::recalcimg(int w, int h)
{
    _bdraw = false;

    if ( image() != NULL )
    {
        _isInResize = true;

        Fl_Image* refimg = image();
        if ( refimg != NULL )
        {
            if ( ( _orgw == w ) && _orgh == h )
            {
                if ( _cachedimg != NULL )
                {
                    delete _cachedimg;
                    _cachedimg = NULL;
                }

                _putw = _orgw;
                _puth = _orgh;

                if ( _puty < h )
                {
                    _puty = ( h - _puth ) / 2;
                }

                if ( _putw < w )
                {
                    _putx = ( w - _putw ) / 2;
                }

            }
            else
            {
                float ratio = 0.0f;

                // check who is shorter ?
                if ( _orgw != w )
                {
                    ratio = float( w ) / float( _orgw );
                    if ( ratio > 0.0f )
                    {
                        _putw = float( _orgw ) * ratio;
                        _puth = float( _orgh ) * ratio;
                    }
                }

                if ( ( _orgh != h ) && ( _puth >= h ) )
                {
                    ratio = float( h ) / float( _orgh );

                    if ( ratio > 0.0f )
                    {
                        _putw = float( _orgw ) * ratio;
                        _puth = float( _orgh ) * ratio;
                    }
                }

                if ( _putw < w )
                {
                    _putx = ( w - _putw ) / 2;
                }

                if ( _puty < h )
                {
                    _puty = ( h - _puth ) / 2;
                }

                if ( _cachedimg != NULL )
                {
                    delete _cachedimg;
                    _cachedimg = NULL;
                }

                if ( ( _putw > 0 ) && ( _puth > 0 ) )
                {
                    _cachedimg = refimg->copy( _putw, _puth );
                }

            }
        }

        _isInResize = false;
    }

    _bdraw = true;
}

int Fl_MRender::handle(int h)
{
    if ( _eventh != NULL )
    {
        return _eventh->event( this, h );
    }

    return 0;
}
