# FFPlayMod (ffmpeg) with FLTK GUI, for Windows32. 
ffplay modified source codes for using my modified FLTK Windows version and Windows audio system.
And it designed to watching RTSP stream media.

## About this: ##
ffplaymod_fltk is a sample for using ffplay of ffmpeg to converting RGBA8888 pixels from YUV buffer in result of decoding API.<br\>
This sample designed to watching stream video of RTSP server, and it tested in H.264 & AAC set of streaming media.

## Requirement: ##
- Please use my repository source codes for successully compiled.
  - (ffmpeg for MinGW-W64) https://github.com/rageworx/ffmpeg-2.2-mingw
  - (FLTK for Windows GDI & Thread safe)  https://github.com/rageworx/fltk-1.3.3-ts
- Recommended compile and build in MinGW-W64 4.9.x

## Not using SDL ? ##
Right, my version never uses SDL. Actually SDL is not light.

## Why modified FLTK ? ##
Actually FTLK is not designed to drawing fast faced GUI like Media Player in normal mode. And FLTK has some drawing error in multi-threaded application, and multiple flush. So I changing it to possible drawing thread-safe for Windows GDI.

## Why not use official ffmpeg ? ##
In case of Windows gcc - MinGW-W64 - has different case by cause Windows background (it sucks...). And MinGW-W64 (mostely you may installed for posix model) need using 'windows.h' header for compile ffmpeg in M-sys shell. So I changed some source codes to successfully compiled and works well in Windows and MinGW-W64.
