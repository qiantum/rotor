let { min, max, abs, floor, ceil, round, PI, cos, sin } = Math
let PI2 = (Math.PI2 = PI + PI)
let EPSI = (Number.EPSILON = 1 / (1 << 12))
let atan, dist, diff, diffAbs, cross, area

function Rotor({
	N, // 转子顶角数
	E, // 偏心距
	P = (N - 1) * 0.8, // 转子顶半径 / 偏心距
	Q = P, // 转子腰半径 / 偏心距
	BP = 0.1, // 缸体转子间隙 / 偏心距
	tickn = 96, // 圆周步进数
	Rfast = true, // 快速计算转子型线
	size, // 预估像素
}) {
	if ((N |= 0) < 2) throw 'err N'
	let NB = N - 1 // 缸体顶数
	let NS = NB + NB // 冲程数
	let N2 = N + N
	tickn = ceil(tickn / N2 / NB) * N2 * NB // 圆周步进数，转子顶*缸体顶*2 的整倍数
	size = ceil(+size || min(size.clientWidth, size.clientHeight))
	E ??= floor(size / (2.3 * (P + N + 3))) // 偏心距
	let G = E * N // 转子大节圆半径
	let g = G - E // 曲轴小节圆半径
	P = E * (P + N + 2) // 转子顶半径
	Q = E * (Q + N) // 转子腰半径
	BP *= E // 缸体转子间隙

	// 转子、曲轴步进角，均匀
	let Tick_ = (this.Tick_ = [...Array.seq(0, tickn)].map(t => (t / tickn) * PI2))
	let Tick = (this.Tick = Tick_.slice(0, tickn))
	let tPQ = tickn / N2 // 转子顶腰步进
	let TPQ = PI2 / N2 // 转子顶腰夹角
	let tQ = n => (n * tickn) / N // 转子腰点起始步进
	let TQ = n => (n * PI2) / N // 转子腰点起始角
	let tS = s => (s * tickn) / NS // 转子冲程起始步进
	let TS = s => (s * PI2) / NS // 转子冲程起始角

	// 曲轴心 X=0 Y=0
	let GX = T => E * cos(T * N) // 转子心X
	let GY = T => E * sin(T * N) // 转子心Y
	let PX = (T, n = 0, p = P) => GX(T) + p * cos(T + TQ(n) + TPQ) // 转子顶X
	let PY = (T, n = 0, p = P) => GY(T) + p * sin(T + TQ(n) + TPQ) // 转子顶Y
	let QX = (T, n = 0, q = Q) => GX(T) + q * cos(T + TQ(n)) // 转子腰X
	let QY = (T, n = 0, q = Q) => GY(T) + q * sin(T + TQ(n)) // 转子腰X

	// 缸体型线  E*cos(T*N-PI) + (P+BP)*cos(T), E*cos(T*N-PI) + (P+BP)*cos(T)
	let BB = Tick_.map(T => [PX(T - TPQ, 0, P + BP), PY(T - TPQ, 0, P + BP)])
	// 缸体步进角，非均匀
	let BT = BB.map(([X, Y]) => atan(X, Y))
	BT[BT.length - 1] = PI2
	// 缸体腰线、顶线步进
	let BQt = [...Array.seq(-tPQ, tPQ, tickn)]
	let BPt = [...Array.seq(tS(1) - tPQ, tS(1) + tPQ)]
	// 缸体腰线、顶线步进角
	let BQT = BQt.map(BT.At())
	let BPT = BPt.map(BT.At())
	// 缸体冲程线步进
	let BSt = [...Array.seq(0, max(NS - 1, 3))].map(s => [
		...Array.seq(tS(s) - tPQ, tS(s + 1) + tPQ, tickn, true),
	])

	let TPB = (T, n) => atan(PX(T, n), PY(T, n)) // 转子顶对应缸体角
	let tPB = (T, n, dir) => TPB(T, n).bfind(BB, [2], dir) // 转子顶对应缸体步进

	// 缸体对转子旋转
	function* RBT(B, TT) {
		if (typeof B == 'number')
			for (let T of TT) {
				let X = P * cos(B + T) - E * cos(B * N + T) - E * cos(T * (N - 1))
				let Y = P * sin(B + T) - E * sin(B * N + T) + E * sin(T * (N - 1))
				yield [atan(X, Y), dist(X, Y), X, Y] // 角[0,PI2) B==0 沿T严格递增 B>0 沿T循环严格递增
			}
		else for (let b of B) yield RBT(b, TT)
	}
	// 转子型线、即缸体绕转子心的内包络线
	let R = Rfast
		? MinDot(RBT(BQt.map(Tick.At()), Tick), t => (t % tPQ ? null : t % (tickn / N) ? P : Q))
		: MinCurve(RBT(Tick, Tick))
	console._`R in ${R.count} details`

	// 工作容积，总容积，总体积
	let V, K, VV, KK, VB, KB
	{
		let v = area(BQt.map(BB.At())) - area(R(0, BQt))
		V = area(BPt.map(BB.At())) - area(R(TS(1), BQt)) - v
		VB = area(BB)
		VV = VB - area(R(0))
		;(K = V / v + 1), (KK = VV / V), (KB = VB / V)
		console._`Vmin ${v}{} Vmax ${V}{} K ${K}{1}  KK ${KK}{1} KB ${KB}{1}`
	}

	size = BB.reduce((v, [X, Y]) => max(v, abs(X), abs(Y)), 0)
	Object.assign(this, { N, E, G, g, P, Q, BP, R, V, K, KK, KB, TQ, TS, size })

	let SS = BSt.map((BSt, s) => {
		function* RR(T) {
			for (let [X, Y] of R(T, BQt)) yield [atan(X, Y), dist(X, Y), X, Y]
		}
		function* TT() {
			for (let t of Array.seq(tS(s), tS(s + 1))) yield RR(Tick_[t])
		}
		// return [...TT()].flatMap((r, t) => { //调试线集
		// 	let s = [...r].map(([A, R, X, Y]) => [X, Y]); return t % 2 ? s.reverse() : s })
		let rs = [...MinDot(TT(), null, BT, BSt)(0, BSt, 0, 0)].reverse()
		return BSt.map(BB.At()).concat(rs).close()
	})

	this.$ = ({ canvas, midx, midy, param }) => {
		let $ = canvas.getContext('2d')
		if (param)
			param.textContent =
				_`N${N}{} E${E}{}\nP${P / E}{1} Q${Q / E}{1}\n` + _`K${K}{} ${KK}{1} ${KB}{1}`
		let x = midx ?? canvas.clientWidth / 2 // 曲轴心X
		let y = midy ?? canvas.clientHeight / 2 // 曲轴心Y

		function $$({ color = '#000', opa = '', thick = 1 } = {}, fill) {
			$.beginPath(), ($.lineWidth = thick)
			opa.length == 1 && /#....../.test(color) && (opa += opa)
			fill ? ($.fillStyle = color + opa) : ($.strokeStyle = color + opa)
		}
		function $$$(fill) {
			fill ? $.fill() : $.stroke(), ($.lineWidth = 1)
			fill ? ($.fillStyle = '#000') : ($.strokeStyle = '#000')
		}
		// 画曲轴小圆
		function $g(style) {
			$$(style), $.arc(x, y, g, 0, PI2), $$$()
		}
		// 画偏心线
		function $Gg(T, style) {
			$$({ color: '#ccc', thick: 4, ...style })
			$.moveTo(x, y), $.lineTo(x + GX(T), y + GY(T)), $$$()
		}
		// 画转子大圆
		function $G(T, style) {
			$$({ color: '#999', ...style }), $.arc(x + GX(T), y + GY(T), G, 0, PI2), $$$()
		}
		// 画转子大圆凸包
		function $GG(style) {
			$$({ color: '#999', opa: '5', ...style }), $.arc(x, y, G + E, 0, PI2), $$$()
		}
		// 画转子顶
		function $P(T, n = 0, O, style) {
			$$({ color: '#00f', ...style })
			BP && $.arc(x + PX(T, n), y + PY(T, n), BP, 0, PI2)
			$.moveTo(x + PX(T, n), y + PY(T, n)), $.lineTo(x + PX(T, n, O), y + PY(T, n, O))
			$$$()
		}
		// 画转子腰
		function $Q(T, n = 0, O, style) {
			$$({ color: '#0f0', ...style })
			$.moveTo(x + QX(T, n), y + QY(T, n)), $.lineTo(x + QX(T, n, O), y + QY(T, n, O))
			$$$()
		}
		// 画转子全部顶
		function $PN(T, O = G, style) {
			for (let n = 0; n < N; n++) $P(T, n, O?.[n] ?? O?.at?.(-1) ?? O, style)
		}
		// 画转子全部腰
		function $QN(T, O = [0, G], style) {
			for (let n = 0; n < N; n++) $Q(T, n, O?.[n] ?? O?.at?.(-1) ?? O, style)
		}
		// 画转子腰凸包
		function $QQ(style) {
			$$({ color: '#9f9', ...style })
			for (let T of Tick_) (T ? $.lineTo : $.moveTo).call($, x + QX(T), y + QY(T))
			$$$()
		}
		// 画缸体
		function $BB(style) {
			$$({ color: '#00f', ...style })
			let to
			for (let [X, Y] of BB) (to = to ? $.lineTo : $.moveTo).call($, x + X, y + Y)
			$$$()
		}
		// 画转子
		function $RR(T, style) {
			$$(style)
			let to
			for (let [X, Y] of R(T)) (to = to ? $.lineTo : $.moveTo).call($, x + X, y + Y)
			$$$()
		}
		// 画冲程区
		function $SS(s, style) {
			$$(style, true)
			let to
			for (let [X, Y] of SS[s]) (to = to ? $.lineTo : $.moveTo).call($, x + X, y + Y)
			$$$(true)
		}
		return Object.assign(
			{ x, y, g: $g, Gg: $Gg, G: $G, GG: $GG },
			{ P: $P, Q: $Q, PN: $PN, QN: $QN, QQ: $QQ, BB: $BB, RR: $RR, SS: $SS }
		)
	}

	// 点集求内包络线 dots:[[ [A, R] ]] mt:正向步进
	function MinDot(dots, fix, TT = Tick_, mt = Tick_.keys()) {
		let M = new Array(tickn).fill(size + size)
		for (let dot of dots)
			for (let [A, R] of dot) {
				let t = (TT == Tick_ ? ceil((tickn * A) / PI2) : A.bfind(TT, null, 1, 0)) % tickn
				M[t] = min(M[t], R)
		}
		mt.values ?? (mt = [...mt])
		M[mt[0]] == size + size && (M[mt[0]] = M[(mt[0] + 1) % tickn])
		M[(mt.at(-1) + 1) % tickn] == size + size && (M[(mt.at(-1) + 1) % tickn] = M[mt.at(-1)])
		M.close()
		for (let t of mt)
			if (t < tickn) M[t] = min(M[t], M[t + 1]) * 0.375 + max(M[t], M[t + 1]) * 0.625
		if (fix) for (let t of mt) if ((d = fix(t, TT[t])) != null) M[t] = d

		function* XY(T, mtt = mt, x = GX(T), y = GY(T)) {
			for (let t of mtt) yield [x + M[t] * cos(T + TT[t]), y + M[t] * sin(T + TT[t])]
		}
		return (XY.count = mt.length - 1), XY
	}
	// 正向曲线集求内包络线 curves:[[ [A, R, X, Y] ]] mt:可闭合
	function MinCurve(curves, __, mt = Tick_.keys()) {
		let M
		for (let cc of curves) {
			cc.values ?? (cc = [...cc])
			if (M != (M ??= cc)) continue // 初始化包络
			cc.forEach(c => (c[4] = c[0].binFind(M, 0, -1)))
			let C = []
			for (let c of cc.keys()) {
				let [a, r, x, y, i] = cc[c]
				let [a_, r_, x_, y_, i_] = cc[c + 1] ?? cc[0]
				i--, (i_ %= M.length) // 遍历曲线连续段，对应包络连续段
				for (let I = i, I1, I_; I != i_; I = I_) {
					let [A, R, X, Y] = M[I]
					let [A_, R_, X_, Y_] = M[(I_ = I1 = I + 1)] ?? M[(I_ = 0)]
					// 求交点和半径
					let [SX, SY, ABc, ABd, cdA, cdB] = cross(X, Y, X_, Y_, x, y, x_, y_)
					if (I_ != i_ && cdB < -EPSI && I_) C.push([1 / 0, 0, 0, 0, I_]) // 去掉长半径包络点
					if (I == i && ABc > EPSI) C.push([a, r, x, y, I1]) // 短半径曲线点
					if (I_ == i_ && ABd > EPSI) C.push([a_, r_, x_, y_, I1]) // 短半径曲线点
					if (SX != null) C.push([atan(SX, SY), dist(SX, SY), SX, SY, I1]) //交点
				}
			}
			C.sort((c, d) => c[4] - d[4] || c[0] - d[0]) // 下标和角度顺序
			let i = 0 // 加减点
			for (let c of C)
				if (c[0] == 1 / 0) M.splice(c[4] + i--, 1)
				else M.splice(c[4] + i++, 0, c), c.length--
			M.close(([A, ...M]) => [A + PI2, ...M]) // 闭合
			i = 0 // 合并重合点
			for (let m of M)
				if (m[0] < M[i][0]) throw 'err a'
				else if (m[0] - M[i][0] > EPSI && abs(m[1] - M[i][1]) > EPSI) M[++i] = m
			M.length = ++i
		}
		mt.values ?? (mt = [...mt])
		function* XY(T, mtt = mt, x = GX(T), y = GY(T)) {
			mtt = mtt.values?.() ?? mtt
			let t = mtt.next().value
			if (t == null) return
			let m = Tick_[t].binFind(M, 0)
			for (let tt of mtt)
				do
					for (let A, TT = Tick_[tt] + (t > tt && PI2); (A = M[m]?.[0]) <= TT; m++)
						yield [x + M[m][1] * cos(T + A), y + M[m][1] * sin(T + A)]
				while (t > (t = tt) && !(m = 0))
		}
		return (XY.count = M.length - 1), XY
	}
}

atan = (x, y) => (Math.atan(y / x) + (x <= 0 ? PI : PI2)) % PI2 // [0,PI2)
dist = (x, y) => Math.sqrt(x * x + y * y)
diff = v => ((v %= PI2) > PI ? v - PI2 : v < -PI ? v + PI2 : v)
diffAbs = v => ((v = abs(v) % PI2) > PI ? PI2 - v : v)
Number.prototype.mod = function (n) {
	return ((this % n) + n) % n
}

Array.prototype.At = function () {
	return this.at.bind(this)
}
Array.prototype.close = function (last = v => (v instanceof Array ? [...v] : v)) {
	return this.push(last ? last(this[0]) : this[0]), this
}
// [from,to]序列
Array.seq = function* (from, to, wrap, oneAll) {
	if (wrap == null) for (; from <= to; from++) yield from
	else
		for (from = from.mod(wrap), to = to.mod(wrap); yield from, oneAll || from != to; )
			(from = (from + 1).mod(wrap)), (oneAll = false)
}
Number.prototype.bfind = function (s, prop, dir, epsi = EPSI) {
	let l = 0,
		h = s.length - 1
	for (let m, c; l <= h; ) {
		;(m = (l + h) >>> 1), (c = this - (prop != null ? s[m][prop] : s[m]))
		if (c >= -epsi && c <= epsi) return m
		;(dir > 0 ? c < 0 : c <= 0) ? (h = m - 1) : (l = m + 1)
	}
	return dir < 0 ? h : dir > 0 ? l : ~l
}

cross = function (ax, ay, bx, by, cx, cy, dx, dy, co) {
	let abc = (ax - cx) * (by - cy) - (ay - cy) * (bx - cx)
	let abd = (ax - dx) * (by - dy) - (ay - dy) * (bx - dx)
	let cda = (cx - ax) * (dy - ay) - (cy - ay) * (dx - ax)
	let cdb = abc - abd + cda
	if (abc * abd >= 0 || cda * cdb >= 0) {
		if (!co || (abc && abd)) return [null, null, abc, abd, cda, cdb]
		return cda == 0 ? [ax, ay, abc, abd, cda, cdb] : [bx, by, abc, abd, cda, cdb]
	}
	let t = cda / (abd - abc)
	return [ax + t * (bx - ax), ay + t * (by - ay), abc, abd, cda, cdb]
}
area = function (s) {
	let [x0, y0] = (s = s.values?.() ?? s).next().value
	let [xx, yy] = [x0, y0]
	let a = 0
	for (let [x, y] of s) (a += (xx - x) * (yy + y)), (xx = x), (yy = y)
	return (a + (xx - x0) * (yy + y0)) / 2
}

Array.prototype.lineHole = function (hole) {
	let ii = this.findIndex(v => v != hole)
	if (ii < 0) throw 'all hole'
	let len = this.length
	let m = 0
	for (let j = ii, i = ii, k, n; (i = (i + 1) % len) != ii; j = i)
		if (this[i] == hole) {
			for (k = i; this[(k = (k + 1) % len)] == hole; );
			for (n = (k - j + len) % len, l = 1; l < n; l++, i = (i + 1) % len)
				this[i] = this[j] + ((this[k] - this[j]) * l) / n
			m += n - 1
		}
	return m
}
