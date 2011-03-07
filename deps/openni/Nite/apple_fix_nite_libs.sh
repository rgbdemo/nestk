#install_name_tool -id "libXnVCNite.dylib" "libXnVCNite.dylib"
#install_name_tool -change "../../Bin/Release/libOpenNI.dylib" libOpenNI.dylib libXnVCNite.dylib
#install_name_tool -change "../../Bin/Release/libXnVNite.dylib" libXnVNite.dylib libXnVCNite.dylib

for lib in libXnVCNite.dylib libXnVFeatures.dylib libXnVHandGenerator.dylib libXnVNite.dylib; do
  echo "Processing $lib"
  install_name_tool -id $lib $lib
  for dep in `otool -L $lib | grep -e '\.\./\.\.' | cut -f 1 -d '(' | sed -e 's/[[:space:]]*//g'`; do
    fullname="$dep"
	name=`basename $dep`
	echo "dep $fullname => $name"
	install_name_tool -change "$fullname" "$name" $lib
  done
done
