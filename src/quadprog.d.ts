declare module 'quadprog' {
  export function solveQP(D: number[][], d: number[], A: number[][], b: number[], meq?: number): { solution: number[]; message: string };
}
declare module '*?url' { const url: string; export default url; }
