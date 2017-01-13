from pudb import set_trace
import pdb
import numpy as np
import matplotlib.pyplot as plt
import scipy.stats

'Change these to your desired paths to the NGSIM data!'
input_folder_80 = "/Users/eugenevinitsky/Box Sync/Research/Bayen/Data" \
           "/NGSIM-Raw/I-80-Main-Data/vehicle-trajectory-data"
# this one is for the US 101 data
input_folder_101= "/Users/eugenevinitsky/Box Sync/Research/Bayen/Data/NGSIM" \
            "/US-101/vehicle-trajectory-data"
output_folder = "/Users/eugenevinitsky/Box Sync/Research/Bayen/Data/" \
            "NGSIM-Processed"
disappear_suffix = "_disappear_headway.txt"
appear_suffix = "_appear_headway.txt"

'''Outputs the NGSIM data for each of the three times.
OUTPUT: pm_4: an array corresponding to all the data from 4pm
        pm_5: an array corresponding to all the data from 5pm
        pm_515: an array corresponding to all the data from 5:15 pm
        The rows of each array are:
        vehicle id, frame_id, lane_id, local_y, mean_speed,
        mean_accel, vehicle_length, vehicle_class, followed_id,
        leader_id
'''

# This reads in the data for I 80
def read_data_80():
    global input_folder

    pm_4 = np.loadtxt(input_folder_80 +
                      "/0400pm-0415pm/trajectories-0400-0415.txt")
    pm_5 = np.loadtxt(input_folder_80 +
                      "/0500pm-0515pm/trajectories-0500-0515.txt")
    pm_515 = np.loadtxt(input_folder_80 +
                        "/0515pm-0530pm/trajectories-0515-0530.txt")

    return pm_4, pm_5, pm_515

''' Read in data that has been processed in the format of extract_cars for I-80
PARAMETERS: NONE
OUTPUT: three sets of data corresponding to 4-4:15, 5-5:15, 5:15-5:30 '''


def read_processed_data_80():
    global output_folder

    pm_4 = np.loadtxt(output_folder+"/4-415-Processed.txt", delimiter=',',
                      skiprows=1)
    pm_5 = np.loadtxt(output_folder+"/5-515-Processed.txt", delimiter=',',
                      skiprows=1)
    pm_515 = np.loadtxt(output_folder+"/515-530-Processed.txt", delimiter=',',
                        skiprows=1)

    return pm_4, pm_5, pm_515

# This reads in the data for I 80
def read_data_101():
    global input_folder

    pm_750 = np.loadtxt(input_folder_101 +
                      "/0750am-0805am/trajectories-0750am-0805am.txt")
    pm_805 = np.loadtxt(input_folder_101 +
                      "/0805am-0820am/trajectories-0805am-0820am.txt")
    pm_820 = np.loadtxt(input_folder_101 +
                        "/0820am-0835am/trajectories-0820am-0835am.txt")

    return pm_750, pm_805, pm_820

''' Read in data that has been processed in the format of extract_cars for I-80
PARAMETERS: NONE
OUTPUT: three sets of data corresponding to 4-4:15, 5-5:15, 5:15-5:30 '''


def read_processed_data_101():
    global output_folder

    pm_750 = np.loadtxt(output_folder+"/750-Processed.txt", delimiter=',',
                      skiprows=1)
    pm_805 = np.loadtxt(output_folder+"/805-Processed.txt", delimiter=',',
                      skiprows=1)
    pm_820 = np.loadtxt(output_folder+"/820-Processed.txt", delimiter=',',
                        skiprows=1)

    return pm_750, pm_805, pm_820


''' Outputs a text file containing only the cars, so no motorcycles
or trucks. We also only extract the left three lanes
PARAMETERS: data_set - The data_set we will process
            file_name - filename we want as our output
Output file has as its columns:
1. Vehicle ID, 2. Frame ID (1/10 of a second)
3. Local X coordinate, 4. Local Y coordinate
5. Global X, 6. Global Y, 7. Lane ID
8. Preceding Vehicle, 9. Following vehicle
10. NGSIM computed space headway, 11. NGSIM Computed time headway
11. Vehicle width '''


def extract_cars(data_set, file_name):
    global output_folder
    f = open(output_folder + file_name, 'w')
    f.write('vehID, fID, locX, locY, globalX, globalY, laneID, PrecedeID, '
            'followID, spaceHeadway, timeHeadway, width\n')
    print data_set.shape[0]
    for i in range(data_set.shape[0]):
        lane_id = data_set[i, 13]
        # check that we are in the left 3 lanes and that we have a car
        if ((data_set[i, 10] == 2) and
                (lane_id == 1 or lane_id == 2 or lane_id == 3)):

            f.write('{0}, {1}, {2}, {3}, {4}, {5}, {6},'
                    '{7}, {8}, {9}, {10}, {11}\n'.
                    format(int(data_set[i, 0]), int(data_set[i, 1]),
                           data_set[i, 4], data_set[i, 5], data_set[i, 6],
                           data_set[i, 7], int(data_set[i, 13]),
                           int(data_set[i, 14]), int(data_set[i, 15]),
                           data_set[i, 16], data_set[i, 17], data_set[i, 9]))
    f.close()

''' Just computes the number of lane changes in a data set '''


def num_lane_changes(data_set):
    num_lane_changes = 0

    'initialize the system'
    veh_id = int(data_set[0, 0])
    lane_id = int(data_set[0, 6])
    for i in range(1, data_set.shape[0]):
        'if we are still on the same vehicle and the lane has changed'
        if int(data_set[i, 0]) == veh_id and lane_id != int(data_set[i, 6]):
            num_lane_changes = num_lane_changes + 1
            print ('vehicle is {0}, lane was {1}, lane is {2} \n'
                   .format(veh_id, lane_id, int(data_set[i, 6])))
        veh_id = int(data_set[i, 0])
        lane_id = int(data_set[i, 6])

    return num_lane_changes

''' Computes the headway at which cars appear and disappear due to
lane change
where the start time of lane changes is determined
according to the method described in Thiemann, Kesting (2008).
The start occurs when the left or right bumper
enters the destination lane. 
PARAMETERS: data_set: the relevant data to work on
            file_prefix: name used for output file
RETURNS: Nothing'''

def lane_change_via_width(data_set, file_prefix):
    global appear_suffix, disappear_suffix
    # first loop through and store the points at which the
    # valid lane change switches occur
    lc_indexes = np.empty([1, 7])
    veh_id = int(data_set[0, 0])
    lane_id = int(data_set[0, 6])

    for i in range(1, data_set.shape[0]):
        # if we are still on the same vehicle and it has switched lanes
        # check that we haven't aborted the lane change within five secs
        # check makes sure the lane change has lasted at least five s
        if (int(data_set[i, 0]) == veh_id and
                lane_id != int(data_set[i, 6]) and
                lane_id != int(data_set[i+50, 6]) and
                veh_id == int(data_set[i+50, 0]) and
                int(data_set[i, 6]) != int(data_set[i-50, 6]) and
                veh_id == int(data_set[i-50, 0])):

                # store the index, the frame, the vehicle id,
                # lane position at lane-change end, lane diff.
                # and vehicle width and new following vehicle
                print i
                temp = np.array([[i, data_set[i, 1],
                                data_set[i, 0], data_set[i, 2],
                                int(data_set[i, 6]) - int(lane_id),
                                data_set[i, 11], data_set[i, 8]]])
                lc_indexes = np.concatenate((lc_indexes, temp), axis=0)

        veh_id = int(data_set[i, 0])
        lane_id = int(data_set[i, 6])

    lc_indexes = np.delete(lc_indexes, 0, 0)

    # now step through and for each item find the crossover
    # since we want to find the start we run backwards
    lc_start = np.empty([1, 3])
    for i in range(lc_indexes.shape[0]):
        for j in range(int(lc_indexes[i, 0]), 0, -1):
            # we are still on the same vehicle
            if int(data_set[j, 0]) == int(lc_indexes[i, 2]):
                horz_shift = data_set[j, 2] - lc_indexes[i, 3]
                # left (right) bumper crosses lane
                if lc_indexes[i, 4] < 0:
                    cond = horz_shift - lc_indexes[i, 5]/2
                else:
                    cond = horz_shift + lc_indexes[i, 5]/2

                # copy over the relevant data i.e.
                # following vehicle prior to lane change, frame, new
                # following vehicle
                temp = np.array([[data_set[j, 8], data_set[j, 1],
                                lc_indexes[i, 6]]])

                # lane change has started
                if ((lc_indexes[i, 4] < 0 and cond > 0) or
                        (lc_indexes[i, 4] > 0 and cond < 0)):
                    lc_start = np.concatenate((lc_start, temp), axis=0)
                    break
            # we have switched cars
            else:
                break
    lc_start = np.delete(lc_start, 0, 0)

    # now we have the frame and the cars whose headways we want
    # Write the headways to file to make this easier
    f = open(output_folder + '/' + file_prefix + disappear_suffix, 'w')
    for i in range(lc_start.shape[0]):
        if int(lc_start[i, 1]) != 0:
            for j in range(data_set.shape[0]):
                # if we have the right frame, vehicle, and headway
                # is within appropriate limits
                if (int(data_set[j, 0]) == int(lc_start[i, 0]) and
                        int(data_set[j, 1]) == int(lc_start[i, 1])):
                    f.write('{0}\n'.format(data_set[j,9]))
        print i
    f.close()


    # now with all the info we have, lets find the headway in
    # the other lane, when the lane change began
    # we use lc_indexes to find the line with the frame when
    # the lane change started and the new following vehicles
    f = open(output_folder + '/' + file_prefix + appear_suffix, 'w')
    for i in range(lc_start.shape[0]):
        for j in range(data_set.shape[0]):
            # check that the vehicle we have found is the new following
            # vehicle, the frame matches, and cutoffs are good
            if (data_set[j, 0] == lc_start[i, 2] and
                    data_set[j, 1] == lc_start[i, 1]):
                f.write('{0}\n'.format(data_set[j,9]))
        print i
    f.close()
    return 


'''We go to the point at which the lane change is
identified in the data, go back 3, 4, 5 seconds
and compute the headway from the follow car at those points.
We then plot all three sets of headways. The choice
of number is based upon the statistic that the
average time of lane change is 4 +/- 2.31 from
Estimating Acceleration and Lane-Changing
Dynamics from Next Generation Simulation
Trajectory Data
PARAMETERS: dataset - the processed dataset from which we extract data
OUTPUT: Three histograms of headways at time of initiated lane change '''


def informal_headway_extraction(data_set):
    h3 = []
    h4 = []
    h5 = []
    'initialize the system'
    veh_id = int(data_set[0, 0])
    lane_id = int(data_set[0, 6])

    'matrix of vehicle id and frame'
    'Here we are computing the probability of a car'
    'lane changing out based on the headway behind it'
    search_mat = data_set[:, 0:2]
    for i in range(1, data_set.shape[0]):
        # if we are still on the same vehicle and it has switched lanes
        # check that we haven't aborted the lane change within five secs
        # check makes sure the lane change has lasted at least five s

        if (int(data_set[i, 0]) == veh_id and
                lane_id != int(data_set[i, 6]) and
                lane_id != int(data_set[i+50, 6]) and
                veh_id == int(data_set[i+50, 0]) and
                int(data_set[i, 6]) != int(data_set[i-50, 6])):

            print ('vehicle is {0}, lane was {1}, lane is {2}'
                   .format(veh_id, lane_id, int(data_set[i, 6])))
            print 'the following vehicle is {0}'.format(int(data_set[i, 8]))
            # if there is an observed following vehicle in the old lane
            if int(data_set[i-1, 8]) != 0:

                # find the frame at which the car is actually in the new lane
                lane_change_frame = int(data_set[i, 1])
                print 'lane change frame is {0}'.format(lane_change_frame)

                # id of the car behind it since we are
                # concerned about its probability of observing a lane change
                follow_id = int(data_set[i-1, 8])

                # frame of the leading car when it "initiated" the lane change
                frame_min3 = int(lane_change_frame) - 30
                frame_min4 = int(lane_change_frame) - 40
                frame_min5 = int(lane_change_frame) - 50
                # find the vehicle and frame corresponding to this frame
                for j in range(1, data_set.shape[0]):

                    # we've found the following car
                    if int(search_mat[j, 0]) == follow_id:

                        # find the frame where the lane change happened
                        if int(search_mat[j, 1]) == frame_min3:
                            h3.append(data_set[j, 3] - data_set[i-30, 3])
                            print ('the three headway is {0}'
                                   .format(data_set[j, 3] - data_set[i-30, 3]))
                            break
                        elif int(search_mat[j, 1]) == frame_min4:
                            h4.append(data_set[j, 3] - data_set[i-40, 3])
                            print ('the four headway is {0}'
                                   .format(data_set[j, 3] - data_set[i-40, 3]))
                        elif int(search_mat[j, 1]) == frame_min5:
                            h5.append(data_set[j, 3] - data_set[i-50, 3])
                            print ('the five headway is {0}'
                                   .format(data_set[j, 3] - data_set[i-50, 3]))
            print ''
        # update the id's used for testing the condition
        veh_id = int(data_set[i, 0])
        lane_id = int(data_set[i, 6])

    print h3
    print h4
    print h5
    plt.subplot(3, 1, 1)
    plt.hist(h3, bins='auto')
    plt.title("headway at 3, 4, 5 seconds, 5:15-5:30pm")
    plt.xlabel("headway")
    plt.ylabel("counts")
    plt.subplot(3, 1, 2)
    plt.hist(h4, bins='auto')
    plt.xlabel("headway")
    plt.ylabel("counts")
    plt.subplot(3, 1, 3)
    plt.hist(h5, bins='auto')
    plt.xlabel("headway")
    plt.ylabel("counts")
    plt.show()

'''We go to the point at which the lane change
is identified in the data, go back 3, 4, 5 seconds
and compute the headway of the new following car
to its predecessor at those times.
We then plot all three sets of headways.
Trajectory Data
PARAMETERS: dataset - the processed dataset from which we extract data
OUTPUT: Three histograms of headways at time of initiated lane change '''


def lane_change_in_headway(data_set):
    headway_3 = []
    headway_4 = []
    headway_5 = []

    'initialize the system'
    veh_id = int(data_set[0, 0])
    lane_id = int(data_set[0, 6])

    'matrix of vehicle id and frame'
    'Here we are computing the probability of a car'
    'lane changing out based on the headway behind it'

    for i in range(1, data_set.shape[0]):
        # if we are still on the same vehicle and it has switched lanes
        # check that we haven't aborted the lane change within five secs
        # check makes sure the lane change has lasted at least five s
        if (int(data_set[i, 0]) == veh_id and
                lane_id != int(data_set[i, 6]) and
                lane_id != int(data_set[i+50, 6]) and
                veh_id == int(data_set[i+50, 0]) and
                int(data_set[i, 6]) != int(data_set[i-50, 6]) and
                veh_id == int(data_set[i-50, 0])):

            # if there is an observed following vehicle
            if int(data_set[i, 8]) != 0 and int(data_set[i, 7]) != 0:

                print 'vehicle is {0}, lane was {1}, ' \
                    'lane is {2}'.format(veh_id, lane_id, int(data_set[i, 6]))
                print ('the new following vehicle is {0}'
                       .format(int(data_set[i, 8])))
                print 'the leading vehicle is {0}'.format(int(data_set[i, 7]))
                # find the time at which the car is actually in the new lane
                lane_change_frame = int(data_set[i, 1])
                print 'lane change frame is {0}'.format(lane_change_frame)

                # id of the car behind it and in front after the lane change
                # Technically if there have been two lane changes
                # into this lane this will yield an error
                follow_id = int(data_set[i, 8])
                lead_id = int(data_set[i, 7])

                # the frame at which the leading car when
                # it "initiated" the lane change
                frame_min3 = int(lane_change_frame) - 30
                frame_min4 = int(lane_change_frame) - 40
                frame_min5 = int(lane_change_frame) - 50

                # now we need to find the preceeding and following vehicle
                # positions at these frames
                for j in range(1, data_set.shape[0]):

                    # we've found the following car
                    if (int(data_set[j, 0]) == follow_id and
                            int(data_set[j, 7] == lead_id)):

                        # find the frame where the lane change happened
                        if int(data_set[j, 1]) == frame_min3:
                            headway_3.append(data_set[j, 9])
                        elif int(data_set[j, 1]) == frame_min4:
                            headway_4.append(data_set[j, 9])
                        elif int(data_set[j, 1]) == frame_min5:
                            headway_5.append(data_set[j, 9])

            print ''
        # update the id's used for testing the condition
        veh_id = int(data_set[i, 0])
        lane_id = int(data_set[i, 6])

    plt.subplot(3, 1, 1)
    plt.hist(headway_3, bins='auto')
    plt.title("headway at 3, 4, 5 seconds, 4:00-4:15pm")
    plt.xlabel("headway")
    plt.ylabel("counts")
    plt.subplot(3, 1, 2)
    plt.hist(headway_4, bins='auto')
    plt.xlabel("headway")
    plt.ylabel("counts")
    plt.subplot(3, 1, 3)
    plt.hist(headway_5, bins='auto')
    plt.xlabel("headway")
    plt.ylabel("counts")
    plt.show()

''' This function makes a histogram of ALL of the headways that have
occured throughout the data set
PARAMETERS: Processed data set
RETURNS: histogram bins
         relevant headways sorted into bins'''


def plot_headways(data_set):
    headways = []
    for i in range(data_set.shape[0]):
        if int(data_set[i, 9]) != 0 and data_set[i, 9] < 400:
            headways.append(data_set[i, 9])

    n, bins, patches = plt.hist(headways, bins='auto')
    plt.show()
    return (n, bins)

''' This function extracts the headways, then computes good bins
for a histogram of this data. It uses these bins to bin the total headways
for the system and then use these to normalize the data
    PARAMETERS: bin_number - how many bins we want for the lane change
    extraction histogram
                u_cutoff - cutoff on the headway values to eliminate
                            unusually large headways
    OUTPUT: plot of the normalized histogram for exiting lane changes'''


def normalized_distribution(data_set, bin_number, u_cutoff=400, l_cutoff=0):
    d_bins, d_headways, a_bins, a_headways = lane_change_via_width(
        data_set, bin_number, u_cutoff, l_cutoff)
    headways = []
    for i in range(data_set.shape[0]):
        if (int(data_set[i, 9]) != 0 and data_set[i, 9] < u_cutoff and
                data_set[i, 9] > l_cutoff):
            headways.append(data_set[i, 9])

    # Operation for car disappearing
    # Bin the headways
    plt.subplot(2, 1, 1)
    t_n, t_bins, t_patches = plt.hist(headways, bins=d_bins)
    plt.subplot(2, 1, 2)
    e_n, e_bins, e_patches = plt.hist(d_headways, bins=d_bins)

    # normalize the lists
    # If there's a more normal python way to do this, that'd be great
    # but in lieu of that, here's some nonsense
    div_list = [a/float(b) for a, b in zip(e_n, t_n)]
    ind = np.arange(len(t_n))
    fig, ax = plt.subplots()
    width = .35
    ax.bar(ind + width, div_list, width, color='r')
    # create the labels
    label_list = []
    for i in range(len(e_bins)-1):
        label_list.append('{:.2f}'.format(e_bins[i]))
    ax.set_xticklabels(tuple(label_list))
    ax.set_xticks(ind + width)
    plt.show()

    # Operation for car appearing
    # Bin the headways
    plt.subplot(2, 1, 1)
    t_n, t_bins, t_patches = plt.hist(headways, bins=a_bins)
    plt.subplot(2, 1, 2)
    e_n, e_bins, e_patches = plt.hist(a_headways, bins=a_bins)

    # normalize the lists
    # If there's a more normal python way to do this, that'd be great
    # but in lieu of that, here's some nonsense
    div_list = [a/float(b) for a, b in zip(e_n, t_n)]
    ind = np.arange(len(t_n))
    fig, ax = plt.subplots()
    width = .35
    ax.bar(ind + width, div_list, width, color='r')
    # create the labels
    label_list = []
    for i in range(len(e_bins)-1):
        label_list.append('{:.2f}'.format(e_bins[i]))
    ax.set_xticklabels(tuple(label_list))
    ax.set_xticks(ind + width)
    plt.show()


''' Takes a set of headway and the name of a distribution and fits
    it over the normalized histogram '''
def headway_fitting(headway, dist_name, u_cutoff, l_cutoff):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.hist(headway, bins='auto', normed=True, alpha=0.2)
    dist = getattr(scipy.stats, dist_name)
    param = dist.fit(headway)
    x = np.linspace(l_cutoff, u_cutoff, 100)
    ax.plot(x, dist.pdf(x, param[0], loc=param[1], scale=param[2]), 'r--')

    return param

def fit_dist(data_set, file_prefix, dist_name, 
            bin_number, u_cutoff=400, l_cutoff=1):
    d_headways = np.loadtxt(output_folder + '/' + 
                            file_prefix)
    # filter
    d_headways = d_headways[np.where(d_headways < u_cutoff)]
    d_headways = d_headways[np.where(d_headways > l_cutoff)]
    s1 = headway_fitting(d_headways, dist_name, u_cutoff, l_cutoff)

    total_headways = []
    # way too much data so don't use all of it, it'll take forever
    for i in range(int(data_set.shape[0])):
        if (int(data_set[i, 9]) != 0 and data_set[i, 9] < u_cutoff and
            data_set[i, 9] > l_cutoff):
            total_headways.append(data_set[i, 9])
    
    s2 = headway_fitting(total_headways, dist_name, u_cutoff, l_cutoff)
    print s1, s2

    plt.show()

    # are these distributions the same via kolmogorov smirnov
    print scipy.stats.kstest(d_headways, dist_name, s1)
    print scipy.stats.kstest(total_headways, dist_name, s2)
    print scipy.stats.ks_2samp(d_headways, total_headways)

if __name__ == '__main__':
    # pm_4, pm_5, pm_515 = read_data_80()
    # extract_cars(pm_4, '/4-415-Processed.txt')
    # extract_cars(pm_5, '/5-515-Processed.txt')
    # extract_cars(pm_515, '/515-530-Processed.txt')
    # pm_4, pm_5, pm_515 = read_processed_data_80()
    # pm_750, pm_805, pm_820 = read_data_101()
    # extract_cars(pm_750, '/750-Processed.txt')
    # extract_cars(pm_805, '/805-Processed.txt')
    # extract_cars(pm_820, '/820-Processed.txt')
    pm_750, pm_805, pm_820 = read_processed_data_101()
    # lane_change_in_headway(pm_5)
    # lane_change_via_width(pm_4, 'pm_4')
    # lane_change_via_width(pm_5, 'pm_5')
    # lane_change_via_width(pm_515, 'pm_515')

    # plot_headways(pm_750)
    # normalized_distribution(pm_515, 10, l_cutoff=10, u_cutoff=120)
    fit_dist(pm_750, 'pm_750'+disappear_suffix, 'lognorm', 
             0, u_cutoff=400, l_cutoff=1)

    #plot_headways(pm_750)
