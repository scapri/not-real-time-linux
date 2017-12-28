while true
do
   echo "this is a test"
   date
   true
   echo $((13**99)) 1>/dev/null 2>&1
   if [ -e ~/.stoprunning ]
   then
      echo "$0 stop"
      exit
   fi
done
