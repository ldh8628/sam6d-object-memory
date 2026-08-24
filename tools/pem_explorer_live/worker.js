self.onmessage=event=>{const data=event.data,{size,observed_bits,projected_bits}=data.mask,[h,w]=size;
 const unpack=s=>{const raw=atob(s),out=new Uint8Array(h*w);for(let i=0;i<out.length;i++)out[i]=(raw.charCodeAt(i>>3)>>(i&7))&1;return out};
 const o=unpack(observed_bits),p=unpack(projected_bits),rgba=new Uint8ClampedArray(h*w*4);
 for(let i=0;i<o.length;i++){rgba[i*4]=p[i]?255:0;rgba[i*4+1]=o[i]?255:0;rgba[i*4+2]=(o[i]&&p[i])?255:0;rgba[i*4+3]=(o[i]||p[i])?220:0}
 const strip=(values,width,height,color)=>{const out=new Uint8ClampedArray(width*height*4),finite=values.filter(Number.isFinite),lo=finite.length?Math.min(...finite):0,hi=finite.length?Math.max(...finite):1,span=Math.max(hi-lo,1e-9),grid=values.length===width*height;for(let y=0;y<height;y++)for(let x=0;x<width;x++){const value=values[grid?y*width+x:x],q=Number.isFinite(value)?(value-lo)/span:0,k=(y*width+x)*4;out[k]=color(q,0);out[k+1]=color(q,1);out[k+2]=color(q,2);out[k+3]=255}return {w:width,h:height,rgba:out}};
 const geometry=strip(data.geometry_distance, data.geometry_distance.length,48,(q,c)=>c===0?255*q:c===1?255*(1-q):40);
 const texture=strip(data.texture_similarity,64,32,(q,c)=>c===2?255*q:c===1?180*q:255*(1-q));
 postMessage({mask:{w,h,rgba},geometry,texture},[rgba.buffer,geometry.rgba.buffer,texture.rgba.buffer])};
