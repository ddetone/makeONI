data_path = '../Data/raw/';
dataset = 'basement_0001a';

sceneDir = [data_path dataset];
frameList = get_synched_frames(sceneDir);

% Displays each pair of synchronized RGB and Depth frames.
for ii = 212 : 1 : numel(frameList)
  imgRgb = imread([sceneDir '/' frameList(ii).rawRgbFilename]);
  imgDepthRaw = swapbytes(imread([sceneDir '/' frameList(ii).rawDepthFilename]));
  
  figure(1);
  % Show the RGB image.
  subplot 121;
  imagesc(imgRgb);
  axis off;
  axis equal;
  title('RGB');
  
  % Show the projected depth image.
  imgDepthProj = project_depth_map(imgDepthRaw, imgRgb);
  subplot 122;
  imagesc(imgDepthProj);
  axis off;
  axis equal;
  title('Projected Depth');
  
  
  imgDepthWorld = rgb_plane2rgb_world_nocolor(imgDepthProj);
  
%   [pcloud, dist] = depthToCloud(imgDepthWorld);
  
  fname = [['../Data/pcd/' dataset] '/' frameList(ii).rawRgbFilename];
  fname(end-3:end) = '.pcd';
  disp(fname)
  savepcd(fname, imgDepthWorld);
  
end