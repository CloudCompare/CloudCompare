@echo off
if not exist pdf mkdir pdf
rm pdf/*.pdf
@echo on
pdflatex Documentation.tex -output-directory=pdf
@echo off
@if not exist pdf/Documentation.pdf goto failure
goto end
:failure
@echo on
@echo generation has failed
@echo off
pause
:end
