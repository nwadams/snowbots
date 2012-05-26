rem birds_eye board_w board_h instrinics distortion image_file
@echo off 
echo Remember:
echo * one row of checkers is an odd size board (7)
echo * one row of checkers is an even size board (4)
echo * NOTE THE EDGE WITH (4) BLACK SQUARES FACES THE CAMERA
echo * Only the inner points are tracked, so a checker board
echo   with 7 rows wide by 4 rows long would be input as
echo   .\birds_eye 6 3 Intrinsics.xml Distortion.xml 
rem del H.xml
birds_eye 6 3 Intrinsics.xml Distortion.xml 
pause