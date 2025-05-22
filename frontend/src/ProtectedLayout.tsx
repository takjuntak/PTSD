import { Navigate, Outlet } from 'react-router-dom'

export default function ProtectedLayout() {
  const token = localStorage.getItem('accessToken')
  return token ? <Outlet /> : <Navigate to="/login" replace />
}
