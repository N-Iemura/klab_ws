import c3d

with open('from_c3d/49_04.c3d', 'rb') as handle:
    reader = c3d.Reader(handle)
    for i, (points, analog) in enumerate(reader.read_frames()):
        print('Frame {}: {}'.format(i, points.round(2)))