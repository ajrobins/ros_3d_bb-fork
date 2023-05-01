# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

std_msgs/MultiArrayLayout  layout        # specification of data layout
int64[]           data          # array of data

#These must correspond to the same order the data[] is in
int32[]	   classes	 # array of classes (up to 4)
int32[]	   probabilities # array of probabilties (between 1-100, up to 4 numbers)


