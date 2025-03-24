function index = GetNearAgent(i, agent, N, rmin, F, f)
ii = 1;
index = [];
for j = [1:i-1,i+1:length(agent)]
    for k = 1:N
      if norm(F*(agent(i).posN(k,:) - agent(j).posN(k,:))') < f*rmin
          index{ii} = agent(j).posN;
          ii = ii + 1;
          break;
      end
    end
end

end