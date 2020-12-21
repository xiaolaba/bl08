# bl08 loader fro MC68HC08 MCUs

project derived from http://www.sparetimelabs.com/bl08/bl08.php



forked,   
platform for build, https://github.com/xiaolaba/Openwrt_Ubuntu_20.04_setup   

build script,
```
apt list --upgradable  
sudo apt update  

## clone the source code
git clone https://github.com/xiaolaba/bl08  

## compile the source code
cd ~\bl08
gcc -Wall -std=c99 -pedantic -o bl08 bl08.c

## run the program
./bl08
```

  
  
```
##  cross-compiler to create a Windows executable in Linux
## Installing the cross-compilation
sudo apt-get install mingw-w64

## 32bit
i686-w64-mingw32-gcc -o test.exe test.c

## 64bit
x86_64-w64-mingw32-gcc -o test.exe test.c

```
