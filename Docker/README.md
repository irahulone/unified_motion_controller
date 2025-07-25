# Docker
Contains files for running the project in Docker container.

## Before Using
Make sure you have Docker Desktop installed and running on your machine. If you want to use dev containers get the dev-containers
extension for VSCode.  
In order to enable GUI applications like rviz and PyQT you need to allow display forwarding.  
#### Windows:
Downlaod an xserver at https://sourceforge.net/projects/vcxsrv/. Ensure it installs at C:\Program Files\ (should be default location)  
Because Windows is dumb it will not copy the symbolic links by default when you clone the github repo causing issues. In order to fix this   
enable developer mode in settings app then use the flag when cloning: "git clone -c core.symlinks=true <repo-url>". You may have to delete your local repo and reclone with the flag.  
#### Linux:
Your good to go!  
#### Mac:   
Not working rn :(.  

## How to Use
Run the appropriate launch script depending on your system and intention. 
"launchmain" - runs the entire project automatically.
"launchdev" - creates dev container to use as development environment.

### Windows:
1. Navigate to Docker directory
2. Enable script execution if you haven't before: Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
3. run .ps1 file: .\launch{main/dev}.ps1

### Linux/Mac:
1. Navigate to Docker directory
2. Enable script execution if you haven't before: chmod +x launch{main/dev}.sh
3. run .sh file: ./launch{main/dev}.sh
