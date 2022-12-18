function theta=angleFinder(q0,q1,q2)

    [x1,y1] = feval(@(x) x{:}, num2cell(q0));
    [x2,y2] = feval(@(x) x{:}, num2cell(q1));
    [x3,y3] = feval(@(x) x{:}, num2cell(q2));

    v_1 = [x2,y2,0] - [x1,y1,0];
    v_2 = [x3,y3,0] - [x2,y2,0];
    theta = rad2deg(atan2(norm(cross(v_1, v_2)), dot(v_1, v_2)));
    hold all
%     plot([q1(1) q0(1)],[q1(2) q0(2)])
%     plot([q2(1) q1(1)],[q2(2) q1(2)])
end
