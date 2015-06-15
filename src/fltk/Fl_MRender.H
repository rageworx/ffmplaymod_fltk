#ifndef __FL_MRENDER_H__
#define __FL_MRENDER_H__

#include <FL/Fl.H>
#include <FL/Fl_Box.H>
#include <Fl/Fl_Image.H>

class Fl_MRender_Event
{
    public:
        virtual int event(void *p, int i) = 0;
};

class Fl_MRender : public Fl_Box
{
    public:
        Fl_MRender(int X, int Y, int W, int H, const char *l=0);
        virtual ~Fl_MRender();

    public:
        void  dodraw( bool b )          { _bdraw = b; }
        bool  dodraw()                  { return _bdraw; }
        void  outstr( const char* str ) { _ostr = (char*)str; }
        char* outstr()                  { return _ostr; }

    public:
        void  eventhandler( Fl_MRender_Event* p );
        Fl_MRender_Event* eventhandler(){ return _eventh; }
        Fl_Image* image() { return Fl_Box::image(); }

    public: /// override
        void draw();
        int  handle(int h);
        void resize(int x, int y, int w, int h);
        void image( Fl_Image* img );

    private:
        void recalcimg(int w, int h);

    protected:
        bool                _bdraw;
        bool                _isInResize;
        unsigned int        _orgw;
        unsigned int        _orgh;
        unsigned int        _putx;
        unsigned int        _puty;
        unsigned int        _putw;
        unsigned int        _puth;
        char*               _ostr;
        Fl_MRender_Event*   _eventh;
        Fl_Image*           _cachedimg;

};

#endif /// of __FL_MRENDER_H__
