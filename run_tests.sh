# Once we have built all of our .so files, this script will run them all in
# testify.
for x in *.so; do
  LD_LIBRARY_PATH=$LD_LIBRARY_PATH:. ./testify $x | grep -v "Counter"
done
