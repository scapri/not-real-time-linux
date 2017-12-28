while true
do
   echo "Running script $0"
   ./busy.sh &
   if [ -e ~/.stoprunning ]
   then
      echo "$0 is stopping"
      exit
   fi
done
