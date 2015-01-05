data_path = '../Data/raw/';
dataset = 'study_0002';


load 'nyu_depth_v2_labeled';

sceneDir = [data_path dataset];
frameList = get_synched_frames(sceneDir);

camera_params;
kc_d = [k1_d, k2_d, p1_d, p2_d, k3_d];
fc_d = [fx_d,fy_d];
cc_d = [cx_d,cy_d]; 

fc_rgb = [fx_rgb,fy_rgb];
cc_rgb = [cx_rgb,cy_rgb]; 
kc_rgb = [k1_rgb,k2_rgb,p1_rgb,p2_rgb,k3_rgb];

% Displays each pair of synchronized RGB and Depth frames.
% matlabpool open
for ii = 1 : 1 : 50
  imgRgb = imread([sceneDir '/' frameList(ii).rawRgbFilename]);
  imgDepth = swapbytes(imread([sceneDir '/' frameList(ii).rawDepthFilename]));


  noiseMask = 255 * double(imgDepth == max(imgDepth(:)));

  % Undistort the noise mask.
  noiseMask = undistort(noiseMask, fc_d, cc_d, kc_d, 0);
  noiseMask = noiseMask > 0;

  imgDepth = undistort_depth(double(imgDepth),fc_d,cc_d,kc_d,0, noiseMask);

  % Fix issues introduced by distortion.
  imgDepth(imgDepth < 600) = 2047;
  imgDepth(noiseMask) = 2047;
  %We did a quick mashup of our API and this to create a basic (but functional) mobile PoS using the phone camera as a scanner.

  depth2 = depth_rel2depth_abs(imgDepth);
  points3d = depth_plane2depth_world(depth2);
  
  fname = [['../Data/pcd/' dataset] '/' frameList(ii).rawRgbFilename];
  fname(end-3:end) = '.pcd';
  disp(sprintf('processed %d of %d frames', ii, numel(frameList)));
  savepcd(fname, points3d);
  
end
% matlabpool close