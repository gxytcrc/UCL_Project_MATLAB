vd = VideoReader('SLAM_Video.mp4');
i = 1;
TF = zeros(10842, 2);
while hasFrame(vd)
    %read the frame of the video
    video = readFrame(vd);
    num = num2str(i);
    %sharpen the image
    video = imsharpen(video);
    %convert image into histogram domain
    [h,s,v]=rgb2hsv(video);
    %increase the contrast
    v = imadjust(v,[0.08,0.8],[0,0.92]);
    %move back to rgb domain
    video=hsv2rgb(h,s,v);
    %resize the image
    video = video(60:1020, 320:1600,:);
    video = imresize(video,[480,640],'nearest');
    if i<10
         imwrite(video,['extend/0000',num, '.jpg']);
    elseif i < 100
         imwrite(video,['extend/000',num, '.jpg']);
    elseif i < 1000
         imwrite(video,['extend/00',num, '.jpg']);
    elseif i < 10000
         imwrite(video,['extend/0',num, '.jpg']);
    else
         imwrite(video,['extend/',num, '.jpg']);
    end
    TF(i, 1) = i;
    TF(i, 2) = vd.currentTime;
    i = i+1;
end
whos video

% img = read(vd,5234);
% img = imsharpen(img);
% [h,s,v]=rgb2hsv(img);
% v = imadjust(v,[0.08,0.8],[0,0.92]);
% img=hsv2rgb(h,s,v);
% img_new = img(60:1020, 320:1600,:);
% img_resize = imresize(img_new,[480,640],'nearest');
% imshow(img_resize);
% imwrite(img_resize,['sharp.png']);