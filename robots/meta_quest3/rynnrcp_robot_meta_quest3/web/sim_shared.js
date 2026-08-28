(function(global){
  const palette=[0x667382,0xb5bdc6,0x8793a1,0xcbd1d7];

  function matrixFromRows(rows){
    const matrix=new THREE.Matrix4();
    matrix.set(...rows.flat());
    return matrix;
  }

  function colorForLink(name){
    const numeric=name.match(/\d+/);
    if(numeric)return palette[Number(numeric[0])%palette.length];
    let hash=0;
    for(const char of name)hash=(hash*31+char.charCodeAt(0))>>>0;
    return palette[hash%palette.length];
  }

  function applyVisual(mesh,visual,transforms){
    const rows=transforms[visual.link_name];
    if(!rows)return;
    mesh.matrix.copy(matrixFromRows(rows))
      .multiply(matrixFromRows(visual.origin_transform));
    mesh.matrixWorldNeedsUpdate=true;
  }

  function attachOrbitControls(renderer,camera,orbit,minRadius,maxRadius){
    let dragging=false,lastX=0,lastY=0;
    function update(){
      const ce=Math.cos(orbit.elevation);
      camera.position.set(
        orbit.target.x+orbit.radius*ce*Math.cos(orbit.azimuth),
        orbit.target.y+orbit.radius*ce*Math.sin(orbit.azimuth),
        orbit.target.z+orbit.radius*Math.sin(orbit.elevation)
      );
      camera.lookAt(orbit.target);
    }
    renderer.domElement.addEventListener("pointerdown",event=>{
      dragging=true;lastX=event.clientX;lastY=event.clientY;
      renderer.domElement.setPointerCapture(event.pointerId);
    });
    renderer.domElement.addEventListener("pointermove",event=>{
      if(!dragging)return;
      orbit.azimuth-=(event.clientX-lastX)*.007;
      orbit.elevation=Math.max(
        -.05,
        Math.min(1.45,orbit.elevation+(event.clientY-lastY)*.007)
      );
      lastX=event.clientX;lastY=event.clientY;update();
    });
    renderer.domElement.addEventListener("pointerup",()=>dragging=false);
    renderer.domElement.addEventListener("wheel",event=>{
      event.preventDefault();
      orbit.radius=Math.max(
        minRadius,
        Math.min(maxRadius,orbit.radius*Math.exp(event.deltaY*.001))
      );
      update();
    },{passive:false});
    renderer.domElement.addEventListener(
      "contextmenu",
      event=>event.preventDefault()
    );
    update();
    return update;
  }

  function resizeRenderer(viewport,renderer,camera){
    const width=viewport.clientWidth,height=viewport.clientHeight;
    if(
      renderer.domElement.width!==Math.round(width*renderer.getPixelRatio())||
      renderer.domElement.height!==Math.round(height*renderer.getPixelRatio())
    ){
      renderer.setSize(width,height,false);
      camera.aspect=width/height;
      camera.updateProjectionMatrix();
    }
  }

  async function postJson(url,body={}){
    const response=await fetch(url,{
      method:"POST",
      headers:{"Content-Type":"application/json"},
      body:JSON.stringify(body)
    });
    const data=await response.json();
    if(!response.ok)throw new Error(data.error||"请求失败");
    return data;
  }

  global.MetaQuestSim={
    applyVisual,
    attachOrbitControls,
    colorForLink,
    matrixFromRows,
    postJson,
    resizeRenderer
  };
})(window);
