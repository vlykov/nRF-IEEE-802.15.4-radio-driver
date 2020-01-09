:: Usage: buildandpack.bat all|nrf5|offline config|noconfig prep|noprep
:: Defaults are all, noconfig, noprep (thus exactly what _build_dox.bat does)

@echo off
if %1.==. ( set target=all ) else ( set target=%1 )
if %2.==. ( set config=noconfig ) else ( set config=%2 )
if %3.==. ( set prep=noprep ) else ( set prep=%3 )

echo Target: %target%
echo Generate config: %config%
echo Prepare deps: %prep%

set RELEASE_SDK=release_hussar

if %prep%==prep (
echo ************************
echo Running prepare_deps ...
echo ************************
call python "../helper_scripts/nrf_automator/main.py" -s s01_sdk_%RELEASE_SDK% s02_prepare_deps s03_ant_prepare_deps s04_release_notes
)

if %config%==config (
echo ***************************************************
echo Running genproject to build *dox_config.h files ...
echo ***************************************************
call python ../helper_scripts/genproject/genproject.py --config "../genproject_config/%RELEASE_SDK%/sdk_genproject.cfg" --save-report-to-json "../../_build/result.res"
)

if not exist "../../_build/nrf5" md "../../_build/nrf5"

if %target%==offline (
   call python "../helper_scripts/nrf_automator/main.py" -s s01_sdk_%RELEASE_SDK% s05_offline_doc"
) else (

  if not exist "buildfiles/tag_files" md "buildfiles/tag_files"

  if %target%==all (
     echo *************************
	 echo Building S112 API dox ...
     doxygen generated/doxygen_s112.doxyfile

     echo Building S113 API dox ...
     doxygen generated/doxygen_s113.doxyfile
	 
     echo Building S132 API dox ...
     doxygen generated/doxygen_s132.doxyfile

     echo Building S140 API dox ...
     doxygen generated/doxygen_s140.doxyfile

     echo Building S212 API dox ...
     doxygen generated/doxygen_s212.doxyfile

	 echo Building S312 API dox ...
	 doxygen generated/doxygen_s312.doxyfile
	 
	 echo Building S332 API dox ...
	 doxygen generated/doxygen_s332.doxyfile
	 
	 echo Building S340 API dox ...
	 doxygen generated/doxygen_s340.doxyfile
     echo *************************
     )

  echo ************************
  echo Building SDK API dox ...
  echo ************************  
  doxygen generated/doxygen_nrf5.doxyfile

  echo ****************
  echo Packing jars ...
  echo ****************
  %COMSPEC% /c ant nrf5 -f generated/buildjars.xml

)

echo DONE && pause
