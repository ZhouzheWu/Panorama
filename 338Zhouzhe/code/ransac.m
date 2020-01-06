function H = HCalculatebyRANSAC(point1, point2)


    assert(all(size(point1) == size(point2)));  % input matrices are of equal size
    assert(size(point1, 2) == 2);  % input matrices each have two columns
    assert(size(point1, 1) >= 4);  % input matrices each have at least 4 rows
    
    %find inliers of by RANSAC
    numIter = 100;
    maxDist = 3;
    onesArr = ones(length(point1), 1);
    point2Hom = [point2 onesArr];
    bestIn = 0;
    for i = 1:numIter
        randFour = randperm(length(point1), 4);
        npoint1 = point1(randFour, :); 
        npoint2 = point2(randFour, :);
        H = HCalculate(npoint1, npoint2);
        ptest = H*point2Hom';
        ptest(3, :) = [];
        dist = sqrt(sum(point1' - ptest).^2);
        inliers = find(dist<maxDist);
        if(size(inliers, 2)>size(bestIn, 2))
            bestIn = inliers;
        end
    end
    H = HCalculate(point1(bestIn,:), point2(bestIn,:));
   
end

function H = HCalculate(point1, point2) %calculate H for maping point2 to point1
    
    assert(all(size(point1) == size(point2)));%point1, point2 are same size matrices
    assert(size(point1, 2) == 2);
    
    n = size(point1, 1);
    if n < 4
        error('Points are not enough');
    end
    H = zeros(3, 3);  % return a 3 x 3 homography matrix

    A = zeros(n*3,9);
    B = zeros(n*3,1);
    for i=1:n
        A(3*(i-1)+1,1:3) = [point2(i,:),1];  %point1(i,:) is corresponds to point2(i,:) for 1:n
        A(3*(i-1)+2,4:6) = [point2(i,:),1];
        A(3*(i-1)+3,7:9) = [point2(i,:),1];
        B(3*(i-1)+1:3*(i-1)+3) = [point1(i,:),1];
    end
    x = (A\B)';
    H = [x(1:3); x(4:6); x(7:9)];

end
