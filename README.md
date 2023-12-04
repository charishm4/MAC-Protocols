# MAC-Protocols

On Ubuntu:
1. Download ns-allinone-2.35.tar.gz
2. Extract the files in the terminal using: tar -zxvf ns-allinone-2.35.tar.gz
3. cd ns-allinone-2.35
./install
4. Edit ~/.bashrc and add the following scripts
   
#LD_LIBRARY_PATH
OTCL_LIB=~/ns-allinone-2.35/otcl-1.14
NS2_LIB=~/ns-allinone-2.35/lib
USR_LOCAL_LIB=/usr/local/lib
export
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$OTCL_LIB:$NS2_LIB:$USR_LOCAL_
#LIB
TCL_LIBRARY
TCL_LIB=~/ns-allinone-2.35/tcl8.5.10/library
USR_LIB=/usr/lib
export TCL_LIBRARY=$TCL_LIB:$USR_LIB
#PATH
XGRAPH=~/ns-allinone-2.35/bin:~/ns-allinone-2.35/tcl8.5.10/unix:~/ns-allinone-2.35/tk8.
5.10/unix
NS=~/ns-allinone-2.35/ns-2.35/
NAM=~/ns-allinone-2.35/nam-1.15/
export PATH=$PATH:$XGRAPH:$NS:$NAM

5. Run the TCL scripts to obtain trace files.
6. Run the AWK files using trace files as:
   awk -f filename.awk filename.tr or 


