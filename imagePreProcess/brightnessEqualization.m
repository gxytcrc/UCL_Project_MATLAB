clear all;
for frame = 5234:5234
    num = num2str(frame);
    if frame<10
        name = ['new_small/0000', num, '.png'];
    elseif frame < 100
         name = ['new_small/000', num,'.png'];
    elseif frame < 1000
         name = ['new_small/00', num,'.png'];
    elseif frame < 10000
         name = ['new_small/0', num,'.png'];
    else
         name = ['new_small/', num,'.png'];
    end
    %chang to the HSV
    img = imread(name);
    [h,s,v]=rgb2hsv(img);
    for i = 1:480
        for j = 1:640
            if ((i - 240)^2 + (j - 320)^2)^0.5 > 220
                v(i,j) = 0.2;
            end
        end
    end
    
    height= min(size(img,1),size(img,2));
    q=sqrt(2);
    
    SIGMA(1)=15; SIGMA(2)=80; SIGMA(3)=250;
    G_sum = zeros(480,640);
    %process the guassian filter
    for i = 1:3
        F = fspecial('gaussian',height,SIGMA(i)/q);
        Gaussian_mid= imfilter(v, F, 'replicate');
        G_sum = G_sum+ Gaussian_mid;
    end
        
    %calculate the mean value of the Gaussian filter result
    Gaussian= G_sum/3;  
    
    m=mean(Gaussian(:));
    [w,height]=size(v);
    out=zeros(size(v));
    gama=power(0.5,((m-Gaussian)/m));
    out=(power(v,gama));
    out = out*1.5;
    for i = 1:480
        for j = 1:640
            if ((i - 240)^2 + (j - 320)^2)^0.5 > 220
                out(i,j) = 0;
            end
        end
    end
    
    %change the result back to the rgb domain
    img_final=hsv2rgb(h,s,out);   
    for channel = 1:3
       img_final(:,:,channel) = medfilt2(img_final(:,:,channel)); 
    end
    imshow(img_final);
    %     Size = size(img);
%     img_hsv = rgb2hsv(img);
%     Intensity = img_hsv(:,:,3);
%     max_value = max(max(Intensity))*0.8;
%     for row = 1:Size(1)
%         for col = 1:Size(2)
%             distance = ((row-240)^2 + (col-320)^2)^0.5;
%             if distance > 218
%                 continue;
%             end
%             
%             patch_max = max(max(Intensity(row-15:row+15, col-15:col+15)));
%             img_hsv(row,col,3) = Intensity(row,col)/(patch_max/max_value);
%         end
%     end
%     img_final = hsv2rgb(img_hsv);
%     figure;
%     imshow(img_final);

%     
    if frame<10
         imwrite(img_final,['extend/0000',num, '.png']);
    elseif frame < 100
         imwrite(img_final,['extend/000',num, '.png']);
    elseif frame < 1000
         imwrite(img_final,['extend/00',num, '.png']);
    elseif frame < 10000
         imwrite(img_final,['extend/0',num, '.png']);
    else
         imwrite(img_final,['extend/',num, '.png']);
    end
end