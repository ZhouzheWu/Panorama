function [ im_blended ] = Blender( input1, input2 )
%Bleand seperate picture together by using feathering
   
    %two input RGB image should have same size
    assert(all(size(input1) == size(input2)));
    assert(size(input1, 3) == 3);

    im_blended = zeros(size(input1), 'like', input1);
 
    adjust1 = RGBAjust(input1);
    adjust2 = RGBAjust(input2);
    for i = 1:3
    im_blended(:, :, i) = (adjust1.*input1(:, :, i) + ...
        adjust2.*input2(:, :, i))./(adjust1+adjust2);
    end
end

function adjust = RGBAjust(im_input, epsilon)
    if nargin < 2
        epsilon = 0.001;
    end
    
    bw = im2bw(im_input,0);
    bw = ones(size(bw)) - bw;
    adjust = bwdist(bw, 'euclidean');
    
    %Normalize
    adjust = rescale(adjust, epsilon, 1);    

end
