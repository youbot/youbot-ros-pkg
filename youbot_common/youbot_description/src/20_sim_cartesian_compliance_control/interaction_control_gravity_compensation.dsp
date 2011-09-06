# Microsoft Developer Studio Project File - Name="interaction_control_gravity_compensation" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Console Application" 0x0103

CFG=interaction_control_gravity_compensation - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "interaction_control_gravity_compensation.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "interaction_control_gravity_compensation.mak" CFG="interaction_control_gravity_compensation - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "interaction_control_gravity_compensation - Win32 Release" (based on "Win32 (x86) Console Application")
!MESSAGE "interaction_control_gravity_compensation - Win32 Debug" (based on "Win32 (x86) Console Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "interaction_control_gravity_compensation - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /c
# ADD CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /c
# ADD BASE RSC /l 0x409 /d "NDEBUG"
# ADD RSC /l 0x409 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib /nologo /subsystem:console /machine:I386
# ADD LINK32 kernel32.lib user32.lib gdi32.lib /nologo /subsystem:console /machine:I386

!ELSEIF  "$(CFG)" == "interaction_control_gravity_compensation - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /GZ /c
# ADD CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /YX /FD /GZ /c
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept
# ADD LINK32 kernel32.lib user32.lib gdi32.lib /nologo /subsystem:console /debug /machine:I386 /out:"Debug_interaction_control_gravity_compensation/interaction_control_gravity_compensation.exe" /pdbtype:sept

!ENDIF 

# Begin Target

# Name "interaction_control_gravity_compensation - Win32 Release"
# Name "interaction_control_gravity_compensation - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=".\interaction_control_gravity_compensation.cpp"
# End Source File
# Begin Source File

SOURCE=".\test_interaction_control_gravity_compensation.cpp"
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=".\interaction_control_gravity_compensation.h"
# End Source File
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter "ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe"
# End Group
# Begin Group "Support Files"

# PROP Default_Filter "*.cpp;*.h"
# Begin Source File

SOURCE=.\common\EulerAngles.cpp
# End Source File
# Begin Source File

SOURCE=.\common\EulerAngles.h
# End Source File
# Begin Source File

SOURCE=.\common\MotionProfiles.cpp
# End Source File
# Begin Source File

SOURCE=.\common\MotionProfiles.h
# End Source File
# Begin Source File

SOURCE=.\common\xxfuncs.cpp
# End Source File
# Begin Source File

SOURCE=.\common\xxfuncs.h
# End Source File
# Begin Source File

SOURCE=.\common\xxinteg.cpp
# End Source File
# Begin Source File

SOURCE=.\common\xxinteg.h
# End Source File
# Begin Source File

SOURCE=.\common\xxinverse.cpp
# End Source File
# Begin Source File

SOURCE=.\common\xxinverse.h
# End Source File
# Begin Source File

SOURCE=.\common\xxmatrix.cpp
# End Source File
# Begin Source File

SOURCE=.\common\xxmatrix.h
# End Source File
# Begin Source File

SOURCE=.\common\xxmodel.h
# End Source File
# Begin Source File

SOURCE=.\common\xxtypes.h
# End Source File
# End Group
# End Target
# End Project

