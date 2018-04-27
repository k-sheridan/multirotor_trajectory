function [ maximums, foundRoots ] = calculateMaximums( startCoefficients, derivative, tf)
%UNTITLED5 Summary of this function goes here
%   this function uses a 3X10 coefficient matrix and a derivative count to
%   calculate the maximums of a given set of functions at a given
%   derivative

%calculate the Coefficient matrix for the given derivative
for it = (1:1:derivative)
   startCoefficients(1, 1:(10 - it)) = polyder(startCoefficients(1, 1:(10 - (it - 1))));
   startCoefficients(2, 1:(10 - it)) = polyder(startCoefficients(2, 1:(10 - (it - 1))));
   startCoefficients(3, 1:(10 - it)) = polyder(startCoefficients(3, 1:(10 - (it - 1))));
end

%reset the size of coeffs
Coefficients = startCoefficients(1:3, 1:(10 - derivative));

%find the roots of the next derivative and combine them into a roots vector
rootsX = roots(polyder(Coefficients(1, :)));
rootsY = roots(polyder(Coefficients(2, :)));
rootsZ = roots(polyder(Coefficients(3, :)));

%combine the roots together without repeating for optimization
foundRoots = zeros(1, 2);
index = 1;
for it = (1:1:length(rootsX))
    if isreal(rootsX(it)) && rootsX(it) <= tf && rootsX(it) >= 0
        if ~any(~(rootsX(it) - foundRoots))
            %fprintf('ADDED ROOT\n')
            foundRoots(index) = rootsX(it);
            index = index + 1;
        end
    end
end
for it = (1:1:length(rootsY))
    if isreal(rootsY(it)) && rootsY(it) <= tf && rootsY(it) >= 0
        if ~any(~(rootsY(it) - foundRoots))
            %fprintf('ADDED ROOT\n')
            foundRoots(index) = rootsY(it);
            index = index + 1;
        end
    end
end
for it = (1:1:length(rootsZ))
    if isreal(rootsZ(it)) && rootsZ(it) <= tf && rootsZ(it) >= 0
        if ~any(~(rootsZ(it) - foundRoots))
            %fprintf('ADDED ROOT\n')
            foundRoots(index) = rootsZ(it);
            index = index + 1;
        end
    end
end

maximums = zeros(3, length(foundRoots) + 1);

for it = (1:1:length(foundRoots))
    maximums(:, it) = [polyval(Coefficients(1, :), foundRoots(it));
        polyval(Coefficients(2, :), foundRoots(it));
        polyval(Coefficients(3, :), foundRoots(it))];
end

maximums(:, length(foundRoots) + 1) = [polyval(Coefficients(1, :), tf);
        polyval(Coefficients(2, :), tf);
        polyval(Coefficients(3, :), tf)];

return;

end

