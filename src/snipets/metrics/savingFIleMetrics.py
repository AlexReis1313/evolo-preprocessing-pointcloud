# %%
#for sim iou

folder = "metric_files/projected/" 
gt_file = folder + "SIMgt_OnlyDynamic_timecuts.txt"
detections_file_old = folder + "boundingBoxMetrics_DynamicProjected"

detections_file = folder + "boundingBoxMetrics_DynamicNonProjected"
tracking_horizon=0
detections_file_output = detections_file + "corrected"+ str(tracking_horizon)+ ".txt"
extractDetections(gt_file, detections_file,detections_file_output, tracking_horizon)



# %%
results_txt = detections_file + 'results.txt'

#motp of 0 is perfect - no distance


gtSource = gt_file
# Format: <timeStamp>, <frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <angle>, <x>, <y>, <z>

# Format: <timeStamp>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <angle>, <x>, <y>, <z>...
tSource=detections_file_output

motMetricsEnhancedCalculator(gtSource, tSource, results_txt, "dist") 

# %%
import numpy as np
from collections import defaultdict


#gt_file = "metric_files/sim_OnlyDynamic/SIMgt_OnlyDynamic_timecuts.txt"
# Format: <timeStamp>, <frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <angle>, <x>, <y>, <z>

#detections_file="metric_files/sim_OnlyDynamic/boundingBoxMetrics_OnlyDynamic.txt"
# Format: <timeStamp>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <angle>, <x>, <y>, <z>...
#detections_file_output="metric_files/sim_OnlyDynamic/SIM_OnlyDynamic_interpolated_corrected.txt"
#this code generates a new detection file like: # Format: <frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <conf>, <x>, <y>, <z>...
#the values in detections_file_output are an interpolation of values in detections_file so that the <frame> number (int) corresponds to the same exatc time step of all gt_file frames
# in practise, for each line of gt_file, we search in detections_file for a timeStamp that is less than 20ms off gt_file<timeStamp>. Up to 2 time steps can be found. If more than 2 are found, the one exacly before and after gtfile<timeStamp> are chosen
#with the 2 timeStamps, we interpolate the values <bb_left>, <bb_top>, <bb_width>, <bb_height>, <angle>, <x>, <y>, <z>... to their value at gt_file<timeStamp> taking into account how close to gt_file<timeStamp> each detections_file<timeStamp> is
#these values are writen down in detections_file_output, with format <frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <conf>, <x>, <y>, <z>..., with <frame> being the gt_file<frame>

#remove_duplicates_by_first_two_values(detections_file, 'output.txt')
#detections_file="metric_files/sim_OnlyDynamic/output.txt"

def extractDetections(gt_file, detections_file,detections_file_output, tracking_horizon):
    MAX_DELTA_TIME = 0.2  # 5hz - 200 milliseconds
    # Step 1: Parse GT timestamps
    gt_timestamps = []
    time_stamps_aux=[]
    with open(gt_file, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            timestamp = float(parts[0])
            aux = False
            for time in time_stamps_aux:
                if time==timestamp:
                    aux= True
            if not aux:
                frame = int(parts[1])
                gt_timestamps.append((timestamp, frame))
                time_stamps_aux.append(timestamp)

    # Step 2: Parse detection file and organize by ID
    detections_by_id = defaultdict(list)
    with open(detections_file, 'r') as f:
        for line in f:
            parts = line.strip().split(',')
            timestamp = float(parts[0])
            obj_id = int(parts[1])
            values = list(map(float, parts[2:]))
            detections_by_id[obj_id].append((timestamp, values))

    # Sort detection lists by timestamp
    for det_list in detections_by_id.values():
        det_list.sort(key=lambda x: x[0])

    # Interpolation function
    def interpolate(v1, v2, t1, t2, t_interp):
        if t2 == t1:  # Prevent division by zero
            return v1
        alpha = (t_interp - t1) / (t2 - t1)
        return [v1[i] + alpha * (v2[i] - v1[i]) for i in range(len(v1))]

    # Step 3: For each GT timestamp, interpolate per ID
    with open(detections_file_output, 'w') as out_f:
        for gt_timestamp, frame in gt_timestamps:
            for obj_id, det_list in detections_by_id.items():

                if tracking_horizon!=0:
                    stamp = gt_timestamp - tracking_horizon
                    if stamp<0:
                        continue
                    for obj_id, det_list in detections_by_id.items():
                        # Filter detections within ±20ms
                        nearby = [d for d in det_list if abs(d[0] - stamp) <= MAX_DELTA_TIME]
                        if not nearby:
                            continue  # No detection for this ID at this timestamp         
                        
                        # Use closest
                        nearest = min(nearby, key=lambda x: abs(x[0] - stamp))
                        interp_values = nearest[1]
                        true_tracking_horizon= gt_timestamp - nearest[0]


                        interp_values[5]=interp_values[5] + (true_tracking_horizon *interp_values[8])
                        interp_values[6]=interp_values[6] + (true_tracking_horizon *interp_values[9])
                    
                            
                else:
                    # Filter detections within ±20ms
                    nearby = [d for d in det_list if abs(d[0] - gt_timestamp) <= MAX_DELTA_TIME]
                    if not nearby:
                        continue  # No detection for this ID at this timestamp

                    # Find closest before and after
                    before = [d for d in nearby if d[0] <= gt_timestamp]
                    after = [d for d in nearby if d[0] >= gt_timestamp]

                    if before and after and before[-1][0] != after[0][0]:
                        t1, v1 = before[-1]
                        t2, v2 = after[0]
                        interp_values = interpolate(v1, v2, t1, t2, gt_timestamp)
                    else:
                        # Use closest
                        nearest = min(nearby, key=lambda x: abs(x[0] - gt_timestamp))
                        interp_values = nearest[1]

                out_f.write(f"{frame},{obj_id}," +",".join(f"{v:.3f}" for v in interp_values[:])+ "\n")

                # Write output: <frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <conf>, <x>, <y>, <z>...
                #out_f.write(f"{frame},{obj_id}," +
                #            ",".join(f"{v:.6f}" for v in interp_values[:6]) + f",{conf:.2f}," +
                #            ",".join(f"{v:.6f}" for v in interp_values[6:]) + "\n")



# %%
#CODE FOR VELOCITIES

import numpy as np
from collections import defaultdict

tracking_horizon = 10 #seconds

gt_file = "metric_files/busterWaraps/boatGT_real2.txt"
# Format: <timeStamp>, <frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <angle>, <x>, <y>, <z>

detections_file="metric_files/busterWaraps/bbTracking_realBuster2.txt"
# Format: <timeStamp>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <angle>, <x>, <y>, <z>...
detections_file_output="metric_files/busterWaraps/bbTracking_realBuster2_velocities" + str(tracking_horizon)+ ".txt"
#this code generates a new detection file like: # Format: <frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <conf>, <x>, <y>, <z>...
#the values in detections_file_output are an interpolation of values in detections_file so that the <frame> number (int) corresponds to the same exatc time step of all gt_file frames
# in practise, for each line of gt_file, we search in detections_file for a timeStamp that is less than 20ms off gt_file<timeStamp>. Up to 2 time steps can be found. If more than 2 are found, the one exacly before and after gtfile<timeStamp> are chosen
#with the 2 timeStamps, we interpolate the values <bb_left>, <bb_top>, <bb_width>, <bb_height>, <angle>, <x>, <y>, <z>... to their value at gt_file<timeStamp> taking into account how close to gt_file<timeStamp> each detections_file<timeStamp> is
#these values are writen down in detections_file_output, with format <frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <conf>, <x>, <y>, <z>..., with <frame> being the gt_file<frame>

#remove_duplicates_by_first_two_values(detections_file, 'output.txt')
#detections_file="metric_files/sim_OnlyDynamic/output.txt"
MAX_DELTA_TIME = 0.2  # 5hz - 200 milliseconds

# Step 1: Parse GT timestamps
gt_timestamps = []
time_stamps_aux=[]
with open(gt_file, 'r') as f:
    for line in f:
        parts = line.strip().split(',')
        timestamp = float(parts[0])
        aux = False
        for time in time_stamps_aux:
            if time==timestamp:
                aux= True
        if not aux:
            frame = int(parts[1])
            gt_timestamps.append((timestamp, frame))
            time_stamps_aux.append(timestamp)

# Step 2: Parse detection file and organize by ID
detections_by_id = defaultdict(list)
with open(detections_file, 'r') as f:
    for line in f:
        parts = line.strip().split(',')
        timestamp = float(parts[0])
        obj_id = int(parts[1])
        values = list(map(float, parts[2:]))
        detections_by_id[obj_id].append((timestamp, values))

# Sort detection lists by timestamp
for det_list in detections_by_id.values():
    det_list.sort(key=lambda x: x[0])

# Interpolation function
def interpolate(v1, v2, t1, t2, t_interp):
    if t2 == t1:  # Prevent division by zero
        return v1
    alpha = (t_interp - t1) / (t2 - t1)
    return [v1[i] + alpha * (v2[i] - v1[i]) for i in range(len(v1))]

# Step 3: For each GT timestamp, interpolate per ID
with open(detections_file_output, 'w') as out_f:
    for gt_timestamp, frame in gt_timestamps:
        stamp = gt_timestamp - tracking_horizon
        if stamp<0:
            continue
        for obj_id, det_list in detections_by_id.items():
            # Filter detections within ±20ms
            nearby = [d for d in det_list if abs(d[0] - stamp) <= MAX_DELTA_TIME]
            if not nearby:
                continue  # No detection for this ID at this timestamp         
            
            # Use closest
            nearest = min(nearby, key=lambda x: abs(x[0] - stamp))
            interp_values = nearest[1]
            true_tracking_horizon= gt_timestamp - nearest[0]
            print(true_tracking_horizon)


            interp_values[5]=interp_values[5] + (true_tracking_horizon *interp_values[8])
            interp_values[6]=interp_values[6] + (true_tracking_horizon *interp_values[9])
           
            #change interp_values - these are of pose 5 seconds before, we will now compute them for current, so pose is given by pose + vel x tracking_horizon

            out_f.write(f"{frame},{obj_id}," +",".join(f"{v:.3f}" for v in interp_values[:])+ "\n")

            # Write output: <frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <conf>, <x>, <y>, <z>...
            #out_f.write(f"{frame},{obj_id}," +
            #            ",".join(f"{v:.6f}" for v in interp_values[:6]) + f",{conf:.2f}," +
            #            ",".join(f"{v:.6f}" for v in interp_values[6:]) + "\n")



# %%
gtSource="metric_files/sim_OnlyDynamic/SIMgt_OnlyDynamic_timecuts.txt"


output_file = "output_with_velocities.txt"

# Read input and parse by ID
data_by_id = {}

with open(gtSource, 'r') as f:
    for line in f:
        parts = line.strip().split(',')

        if len(parts) < 12:
            continue  # Skip malformed lines

        timestamp = float(parts[0])
        frame = int(parts[1])
        obj_id = int(parts[2])
        bb_centerX = float(parts[3])
        bb_centerY = float(parts[4])
        bb_width = float(parts[5])
        bb_height = float(parts[6])
        bb_angle = float(parts[7])
        x = float(parts[8])
        y = float(parts[9])
        z = float(parts[10])

        entry = {
            "timestamp": timestamp,
            "frame": frame,
            "id": obj_id,
            "bb_centerX": bb_centerX,
            "bb_centerY": bb_centerY,
            "bb_width": bb_width,
            "bb_height": bb_height,
            "bb_angle": bb_angle,
            "x": x,
            "y": y,
            "z": z
        }

        data_by_id.setdefault(obj_id, []).append(entry)

# Compute velocities and write output
with open(output_file, 'w') as out:

    for obj_id, records in data_by_id.items():
        # Sort by timestamp
        records.sort(key=lambda r: r["timestamp"])

        for i, current in enumerate(records):
            if i == 0:
                # First value: velocity = 0
                velx = 0.0
                vely = 0.0
            else:
                prev = records[i - 1]
                dt = current["timestamp"] - prev["timestamp"]
                if dt > 0:
                    velx = (current["x"] - prev["x"]) / dt
                    vely = (current["y"] - prev["y"]) / dt
                else:
                    velx = 0.0
                    vely = 0.0

            out.write(f"{current['timestamp']:.3f},{current['frame']},{obj_id},"
                      f"{current['bb_centerX']:.2f},{current['bb_centerY']:.2f},"
                      f"{current['bb_width']:.2f},{current['bb_height']:.2f},{current['bb_angle']:.2f},"
                      f"{current['x']:.2f},{current['y']:.2f},{current['z']:.2f},"
                      f"{velx:.3f},{vely:.3f}\n")

# %%
def motMetricsEnhancedCalculator(gtSource, tSource, results_txt, cost):
  # import required packages
  import motmetrics as mm
  import numpy as np
  results_file = open(results_txt, 'w')

  # load ground truth
  gt = np.loadtxt(gtSource, delimiter=',')

  # load tracking output
  t = np.loadtxt(tSource, delimiter=',')

  # Create an accumulator that will be updated during each frame
  acc = mm.MOTAccumulator(auto_id=True)

  # Max frame number maybe different for gt and t files
  for frame in range(int(gt[:,1].max())):
    frame += 1 # detection and frame numbers begin at 1

    # select id, x, y, width, height for current frame
    # required format for distance calculation is X, Y, Width, Height \
    # We already have this format
    gt_dets = gt[gt[:,1]==frame,2:10] # select all detections in gt
    t_dets = t[t[:,0]==frame,1:9] # select all detections in t
    if cost=="IOU":
      C = mm.distances.iou_matrix(gt_dets[:,1:6], t_dets[:,1:6], \
                                  max_iou=0.5) # format: gt, t
    # Format: <frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <conf>, <x>, <y>, <z>

    #  for iou we need:           
    #   # Format: <frame>, <id>, <bb_centerX>, <bb_centerY>, <bb_width>, <bb_height>, <bb_angle>, <x>, <y>, <z>, <velx>,<vely>
 
    else:
      C=mm.distances.norm2squared_matrix(gt_dets[:,6:8], t_dets[:,6:8])
      C=np.sqrt(C)
    # Call update once for per frame.
    # format: gt object ids, t object ids, distance
    acc.update(gt_dets[:,0].astype('int').tolist(), 
              t_dets[:,0].astype('int').tolist(), C)

  mh = mm.metrics.create()

  summary = mh.compute(acc, metrics=['num_frames', 'idf1', 'idp', 'idr', \
                                     'recall', 'precision', 'num_objects', \
                                     'mostly_tracked', 'partially_tracked', \
                                     'mostly_lost', 'num_false_positives', \
                                     'num_misses', 'num_switches', \
                                     'num_fragmentations', 'mota', 'motp' \
                                    ], \
                      name='acc')

  strsummary = mm.io.render_summary(
      summary,
      #formatters={'mota' : '{:.2%}'.format},
      namemap={'recall': 'Rcll', 'precision': 'Prcn', 'num_objects': 'GT', \
               'num_false_positives': 'FP', \
               'num_misses': 'FN', 'num_switches' : 'IDsw', \
               'num_fragmentations' : 'FM', 'mota': 'MOTA', 'motp' : 'MOTP',  \
              }
  )

  results_file.write(strsummary)
  results_file.close()
  print(strsummary)

# %%
results_txt = 'results_test.txt'

#motp of 0 is perfect - no distance


gtSource = "metric_files/busterWaraps/boatGT_real2.txt"
# Format: <timeStamp>, <frame>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <angle>, <x>, <y>, <z>

# Format: <timeStamp>, <id>, <bb_left>, <bb_top>, <bb_width>, <bb_height>, <angle>, <x>, <y>, <z>...
tSource="metric_files/busterWaraps/bbTracking_realBuster2_velocities3.txt"

motMetricsEnhancedCalculator(gtSource, tSource, results_txt, "dist") 

# %%
results_txt = 'results.txt'
gtSource="metric_files/sim_OnlyDynamic/SIMgt_OnlyDynamic_timecuts.txt"
tSource="metric_files/sim_OnlyDynamic/SIM_OnlyDynamic_interpolated_corrected.txt"


motMetricsEnhancedCalculator(gtSource, tSource, results_txt, "IOU") 

# %%


# %%


# %%
motMetricsEnhancedCalculator(gtSource, tSource, results_txt, "IOU") 

# %% [markdown]
# I still need code that takes timestamps and connects to most probable gt timestamp

# %%
def remove_duplicates_by_first_two_values(input_file, output_file):
    seen_keys = set()
    filtered_lines = []

    with open(input_file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue  # skip empty lines
            parts = line.split(',')
            if len(parts) < 2:
                continue  # skip malformed lines
            key = (parts[0], parts[1])
            if key not in seen_keys:
                seen_keys.add(key)
                filtered_lines.append(line)

    with open(output_file, 'w') as f:
        for line in filtered_lines:
            f.write(line + '\n')

    print(f"Filtered lines written to: {output_file}")

# Usage example:


# %%
import pygame
import sys
from collections import defaultdict

# --- Configuration Parameters (User Definable) ---
# Set the desired frame rate for the visualization (frames per second)
FRAME_RATE = 10

# Set the zoom level. A higher value means objects appear larger.
# For example, 1.0 is no zoom, 2.0 makes everything twice as big.
ZOOM_LEVEL = 5.0

# Window dimensions
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 1000

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
COLORS = [RED, GREEN, BLUE, (255, 255, 0), (0, 255, 255), (255, 0, 255), (128, 0, 0), (0, 128, 0), (0, 0, 128)]

# --- Data Loading and Parsing ---
def load_mot16_data(filepath):
    """
    Loads bounding box data from a MOT16 format text file.
    Organizes the data by frame ID.

    Args:
        filepath (str): The path to the MOT16 format text file.

    Returns:
        dict: A dictionary where keys are frame IDs and values are lists of
              dictionaries, each representing a bounding box for that frame.
              Each bounding box dictionary contains:
              'id', 'bb_left', 'bb_top', 'bb_width', 'bb_height',
              'conf', 'x', 'y', 'z'.
    """
    frames_data = defaultdict(list)
    try:
        with open(filepath, 'r') as f:
            for line in f:
                parts = line.strip().split(',')
                if len(parts) >= 10: # Ensure enough parts for MOT16 format
                    frame_id = int(parts[0])
                    obj_id = int(parts[1])
                    bb_left = float(parts[2])
                    bb_top = float(parts[3])
                    bb_width = float(parts[4])
                    bb_height = float(parts[5])
                    conf = float(parts[6])
                    x = float(parts[7])
                    y = float(parts[8])
                    z = float(parts[9])

                    frames_data[frame_id].append({
                        'id': obj_id,
                        'bb_left': bb_left,
                        'bb_top': bb_top,
                        'bb_width': bb_width,
                        'bb_height': bb_height,
                        'conf': conf,
                        'x': x,
                        'y': y,
                        'z': z
                    })
    except FileNotFoundError:
        print(f"Error: File not found at {filepath}")
        sys.exit(1)
    except ValueError as e:
        print(f"Error parsing line: {line.strip()} - {e}")
        sys.exit(1)
    return frames_data

# --- Main Visualization Logic ---
def main():
    """
    Main function to initialize Pygame, load data, and run the visualization loop.
    """
    pygame.init()

    # Set up the display
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("MOT16 Bounding Box Viewer")

    # Load data
    data_filepath = 'testgt.txt'
    frames_data = load_mot16_data(data_filepath)

    if not frames_data:
        print("No data loaded. Exiting.")
        pygame.quit()
        sys.exit()

    # Get sorted list of frame IDs
    sorted_frame_ids = sorted(frames_data.keys())
    if not sorted_frame_ids:
        print("No frames found in data. Exiting.")
        pygame.quit()
        sys.exit()

    # Calculate initial center offset based on the first frame's objects
    first_frame_id = sorted_frame_ids[0]
    first_frame_objects = frames_data[first_frame_id]

    if first_frame_objects:
        # Calculate average center of objects in the first frame
        total_x = 0
        total_y = 0
        for obj in first_frame_objects:
            center_x = obj['bb_left'] + obj['bb_width'] / 2
            center_y = obj['bb_top'] + obj['bb_height'] / 2
            total_x += center_x
            total_y += center_y

        avg_center_x = total_x / len(first_frame_objects)
        avg_center_y = total_y / len(first_frame_objects)

        # Calculate offset to center these objects in the screen
        # We want (avg_center_x, avg_center_y) to map to (SCREEN_WIDTH/2, SCREEN_HEIGHT/2)
        offset_x = SCREEN_WIDTH / 2 - (avg_center_x * ZOOM_LEVEL)
        offset_y = SCREEN_HEIGHT / 2 - (avg_center_y * ZOOM_LEVEL)
    else:
        # If no objects in the first frame, no offset needed (or set to 0)
        offset_x = SCREEN_WIDTH / 2
        offset_y = SCREEN_HEIGHT / 2
        print("Warning: First frame has no objects. Window will be centered at (0,0) relative to its own coordinates.")


    clock = pygame.time.Clock()
    running = True
    current_frame_index = 0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Clear the screen
        screen.fill(BLACK)

        # Get the current frame ID
        current_frame_id = sorted_frame_ids[current_frame_index]
        current_frame_objects = frames_data[current_frame_id]

        # Display current frame ID
        font = pygame.font.Font(None, 36) # Default font, size 36
        text_surface = font.render(f"Frame: {current_frame_id}", True, WHITE)
        screen.blit(text_surface, (10, 10)) # Position at top-left

        # Draw bounding boxes for the current frame
        for obj in current_frame_objects:
            obj_id = obj['id']
            bb_left = obj['bb_left']
            bb_top = obj['bb_top']
            bb_width = obj['bb_width']
            bb_height = obj['bb_height']

            # Assign a color based on object ID for consistent tracking
            color = COLORS[obj_id % len(COLORS)]

            # Apply zoom and offset to coordinates
            # First, apply offset to move the origin, then zoom
            # Or, apply zoom first, then apply offset to the zoomed coordinates
            # Let's do: (original_coord * zoom_level) + offset
            display_left = (bb_left * ZOOM_LEVEL) + offset_x
            display_top = (bb_top * ZOOM_LEVEL) + offset_y
            display_width = bb_width * ZOOM_LEVEL
            display_height = bb_height * ZOOM_LEVEL

            # Draw the rectangle
            pygame.draw.rect(screen, color,
                             (display_left, display_top, display_width, display_height),
                             2) # 2 pixels wide border

            # Optionally, draw object ID next to the bounding box
            id_font = pygame.font.Font(None, 24)
            id_text_surface = id_font.render(f"ID: {obj_id}", True, color)
            screen.blit(id_text_surface, (display_left + 5, display_top + 5))


        # Update the display
        pygame.display.flip()

        # Move to the next frame
        current_frame_index = (current_frame_index + 1) % len(sorted_frame_ids)

        # Control frame rate
        clock.tick(FRAME_RATE)

    pygame.quit()
    sys.exit()

if __name__ == '__main__':
    main()



