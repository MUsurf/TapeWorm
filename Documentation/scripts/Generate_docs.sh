# Must be run from project root directory

mkdir Documentation/Generated
destination_directory="$(pwd)/Documentation/Generated"

cd $(cat Documentation/targets)/src


# Grab python files from packages in catkin directory
for d in */src/*.py ; do
    echo $(ls $d)
    cp $(ls $d) "$destination_directory"
done

